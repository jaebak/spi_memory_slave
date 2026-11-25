// Simulation model of a 256-byte serial memory. Only the four SPI pins
// connect it to the master. Locations have unknown contents until written.
module spi_memory_slave (
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output wire miso
);
    reg [7:0] memory [0:255];
    reg [7:0] command, address, shift_in;
    reg [4:0] bits_received;
    reg       reply_bit;
    wire [7:0] next_byte = {shift_in[6:0], mosi};

    // Mode 0 samples data on rising edges, most significant bit first.
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            bits_received <= 0;
            shift_in <= 0;
            command <= 0;
            address <= 0;
        end else if (bits_received < 24) begin
            shift_in <= next_byte;
            bits_received <= bits_received + 1'b1;
            case (bits_received)
                7:  command <= next_byte;
                15: address <= next_byte;
                23: if (command == 8'h02) memory[address] <= next_byte;
            endcase
        end
    end

    // After rising edge 16, the complete address is available. The next
    // falling edge presents bit 7, ready for the master to sample on edge 17.
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n)
            reply_bit <= 1'b0;
        else if (command == 8'h03 && bits_received >= 16 && bits_received < 24)
            reply_bit <= memory[address][23 - bits_received];
        else
            reply_bit <= 1'b0;
    end

    // Release the shared return wire whenever this device is deselected.
    assign miso = cs_n ? 1'bz : reply_bit;
endmodule
