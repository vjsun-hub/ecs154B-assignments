module muxNto1 #(
    parameter int N = 8,
    parameter int WIDTH = 16 
) ( 
    input logic [N-1:0][WIDTH-1:0] in_bus,
    input logic [$clog2(N)-1:0] sel,
    output logic [WIDTH-1:0] out
);
    
    // TODO: Implement the N to 1 multiplexer logic
    /*
    The parameter N is defined as a parameter in the module. 
    The width of each input and output is defined by the parameter WIDTH. 
    The select signal will not select an input outside the range of N inputs.
    */

    always @(*) begin
        out = in_bus[sel];
    end
    
endmodule
