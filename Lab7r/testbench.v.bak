// Define top-level testbench
`include "constants.h"
// Top level has no inputs or outputs
// It only needs to instantiate CPU, Drive the inputs to CPU (clock, reset)
// and monitor the outputs. This is what all testbenches do


`timescale 1ns/1ps
`define clock_period  10

module cpu_tb;
integer   f, i;
reg       clock, reset;    // Clock and reset signals
wire  [8*26:1] stringvar;


// Instantiate CPU
cpu cpu0(clock, reset);
// Initialization and signal generation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

initial  
  begin 
   clock = 1'b0;       
   reset = 1'b0;  // Apply reset for a few cycles
   #(4.25*`clock_period) reset = 1'b1;
end
     
always 
   #(`clock_period / 2) clock = ~clock;  // Clock generation 

initial begin  // Ta statements apo ayto to begin mexri to "end" einai seiriaka

  // Initialize Data Memory 
  $readmemh("program.hex", cpu0.cpu_IMem.data);

  // Edw, to "program.hex" einai ena arxeio pou prepei na brisketai sto 
  // directory pou trexete th Verilog kai na einai ths morfhs:
  
  // @0    00000000
  // @4    20100009
  // @8    00000000
  // @C    00000000
  // ...
  
  // H aristerh sthlh, meta to @, exei th dieythynsh ths mnhmhs (hex),
  // kai h deksia sthlh ta dedomena sth dieythynsh ayth (pali hex).
  // Sto paradeigma pio panw, oi lekseis stis dieythynseis 0, 8 kai 12
  // einai 0, kai sth dieythynsh 4 exei thn timh 32'h20100009. An o PC
  // diabasei thn dieythynsh 4, h timh ekei exei thn entolh
  //   addi $16, $0, 9

  // To deytero orisma ths $readmemh einai pou akribws brisketai h mnhmh
  // pou tha arxikopoihthei. Sto paradeigma, to "dat0" einai to onoma pou
  // dwsame sto instance tou datapath. To "mem" einai to onoma pou exei
  // to instance ths mnhmhs MESA sto datapath, kai to "data" einai to 
  // onoma pou exei to pragmatiko array ths mhnhs mesa sto module ths.
  // An exete dwsei diaforetika onomata, allakste thn $readmemh.

  // Enallaktika, an sas boleyei perissotero, yparxei h entolh $readmemb
  // me thn akribws idia syntaksh. H aristerh sthlh tou arxeiou exei
  // thn idia morfh (dieythynseis se hex), alla h deksia sthlh exei
  // ta dedomena sto dyadiko. Etsi h add mporouse na einai:

  // @4    00100000000100000000000000001001

  // ... h kai akoma kalytera:
  
  // @4    001000_00000_10000_0000000000001001

  // (h Verilog epitrepei diaxwristika underscores).


  // Termatismos ekteleshs:
  // $finish;

end 



endmodule



module instr2str(instr, stringvar); 
input  [31:0]   instr;
output reg [39:0]  stringvar;

  always@(*)
    if (instr == 32'b0) stringvar = "---";
    else
    case(instr[31:26])
        6'b000000: 
            case (instr[5:0] )
                6'b000000 : stringvar = "SLL";
                6'b000010 : stringvar = "SRL";
                6'b000100 : stringvar = "SLLV";
                6'b000110 : stringvar = "SRLV";
                6'b100000 : stringvar = "ADD";         
                6'b100010 : stringvar = "SUB";
                6'b100100 : stringvar = "AND";
                6'b100101 : stringvar = "OR";
                6'b100111 : stringvar = "NOR";
                6'b101010 : stringvar = "SLT"; 
                default   : stringvar = "---";  
            endcase
      6'b100011: stringvar = "LW";  
      6'b101011: stringvar = "SW";  
      6'b000100: stringvar = "BEQ";  
      6'b000101: stringvar = "BNE";  
      6'b001000: stringvar = "ADDI"; 
      default  : stringvar = "---";  
      endcase

endmodule


