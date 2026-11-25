`timescale 1ns/1ps

module spi_memory_tb;
    reg clk = 0;
    always #41.667 clk = ~clk;  // 12 MHz, rounded to the simulator's picosecond.

    reg rst_n = 0;
    reg start = 0, write_enable = 0;
    reg [7:0] address = 0, write_data = 0;
    wire busy, done;
    wire [7:0] read_data;
    wire spi_cs_n, spi_sclk, spi_mosi, spi_miso;

    spi_memory_controller dut (
        .clk(clk), .rst_n(rst_n), .start(start), .write_enable(write_enable),
        .address(address), .write_data(write_data), .busy(busy), .done(done),
        .read_data(read_data), .spi_cs_n(spi_cs_n), .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso)
    );
    spi_memory_slave memory_device (
        .cs_n(spi_cs_n), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso)
    );

    // Observe the bus independently of both modules. Collect bits exactly
    // as an external instrument would, sampling only on rising serial edges.
    reg [23:0] expected_out, expected_in, observed_out, observed_in;
    integer rising_edges, falling_edges;
    integer requests = 0, frames = 0;
    integer reads = 0, writes = 0;
    reg frame_active = 0;
    realtime last_rise, last_fall;

    always @(negedge spi_cs_n) if (rst_n) begin
        if (spi_sclk !== 1'b0 || frame_active || frames + 1 != requests) begin
            $display("ERROR: Unexpected frame start or non-idle serial clock");
            $finish;
        end
        frame_active = 1;
        rising_edges = 0;
        falling_edges = 0;
        observed_out = 0;
        observed_in = 0;
    end

    always @(posedge spi_sclk) if (rst_n) begin
        if (!frame_active || spi_cs_n !== 1'b0 || rising_edges != falling_edges) begin
            $display("ERROR: Unexpected rising edge with CS=%b", spi_cs_n);
            $finish;
        end
        // The low interval may be longer at a byte boundary while the next
        // byte is submitted. Within a byte each half-period is 500 ns.
        if (rising_edges % 8 != 0 &&
            ($realtime-last_fall < 499.9 || $realtime-last_fall > 500.1)) begin
            $display("ERROR: Wrong low clock interval at bit %0d", rising_edges);
            $finish;
        end
        last_rise = $realtime;
        observed_out = {observed_out[22:0], spi_mosi};
        observed_in  = {observed_in[22:0], spi_miso};
        rising_edges = rising_edges + 1;
    end

    always @(negedge spi_sclk) if (rst_n) begin
        if (!frame_active || spi_cs_n !== 1'b0 || rising_edges != falling_edges+1) begin
            $display("ERROR: Unexpected falling edge with CS=%b", spi_cs_n);
            $finish;
        end
        if ($realtime-last_rise < 499.9 || $realtime-last_rise > 500.1) begin
            $display("ERROR: Wrong high clock interval at bit %0d", rising_edges);
            $finish;
        end
        last_fall = $realtime;
        falling_edges = falling_edges + 1;
    end

    always @(posedge spi_cs_n) if (rst_n) begin
        if (!frame_active || spi_sclk !== 1'b0 || rising_edges != 24 || falling_edges != 24) begin
            $display("ERROR: Bad framing: rising=%0d falling=%0d", rising_edges, falling_edges);
            $finish;
        end
        if (observed_out !== expected_out || observed_in !== expected_in) begin
            $display("ERROR: Frame %0d: MOSI=%h expected=%h MISO=%h expected=%h",
                     frames+1, observed_out, expected_out, observed_in, expected_in);
            $finish;
        end
        if (observed_out[23:16] == 8'h03) reads = reads + 1;
        else writes = writes + 1;
        frames = frames + 1;
        frame_active = 0;
        $display("FRAME %0d MOSI=%06h MISO=%06h rising=%0d falling=%0d",
                 frames, observed_out, observed_in, rising_edges, falling_edges);
        #1;
        if (spi_miso !== 1'bz) begin
            $display("ERROR: Memory did not release MISO");
            $finish;
        end
    end

    // A student can add operations here without driving individual SPI bits.
    // The expected reply is supplied by the test, independently of the model.
    task automatic transfer;
        input       wr;
        input [7:0] addr;
        input [7:0] payload;
        input [7:0] expected_read;
        begin
            @(negedge clk);
            if (busy !== 1'b0 || spi_cs_n !== 1'b1 || spi_miso !== 1'bz) begin
                $display("ERROR: Controller or bus is not idle before request");
                $finish;
            end
            requests = requests + 1;
            expected_out = {wr ? 8'h02 : 8'h03, addr, wr ? payload : 8'h00};
            expected_in = {16'h0000, wr ? 8'h00 : expected_read};
            write_enable = wr;
            address = addr;
            write_data = payload;
            start = 1;
            @(negedge clk);
            start = 0;
            if (busy !== 1'b1) begin
                $display("ERROR: Request was not accepted");
                $finish;
            end
            // Exercise the controller's input registers while the frame is busy.
            write_enable = !wr;
            address = ~addr;
            write_data = ~payload;
            while (done !== 1'b1) @(negedge clk);
            if (busy !== 1'b0 || spi_cs_n !== 1'b1 || frames != requests) begin
                $display("ERROR: Completion before the end of the frame");
                $finish;
            end
            if (!wr && read_data !== expected_read) begin
                $display("ERROR: READ address=%h received=%h expected=%h", addr, read_data, expected_read);
                $finish;
            end
            @(negedge clk);
            if (done !== 1'b0) begin
                $display("ERROR: done exceeded one system clock");
                $finish;
            end
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        // Start with the two frames shown in the student guide.
        transfer(1, 8'h2A, 8'hA5, 0);
        transfer(0, 8'h2A, 0, 8'hA5);
        // Different addresses and an overwrite catch simple loopback,
        // address aliasing, and a slave that only saves the last byte.
        transfer(1, 8'hAA, 8'h3C, 0);
        transfer(0, 8'h2A, 0, 8'hA5);
        transfer(1, 8'h2A, 8'h81, 0);
        transfer(0, 8'hAA, 0, 8'h3C);
        transfer(0, 8'h2A, 0, 8'h81);
        if (frames != requests || reads == 0 || writes == 0) begin
            $display("ERROR: Incomplete exercise");
            $finish;
        end
        $display("SPI_MEMORY_PASS frames=%0d reads=%0d writes=%0d", frames, reads, writes);
        $finish;
    end

    initial begin
        #250000;
        $display("ERROR: Timeout waiting for a complete transaction");
        $finish;
    end
endmodule
