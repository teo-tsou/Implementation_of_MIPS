/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/

module cpu(input clock, input reset);
 reg [31:0] PC; 
 reg [31:0] IFID_PCplus4;
 reg [31:0] IFID_instr;
 reg [31:0] IDEX_rdA, IDEX_rdB, IDEX_signExtend , IDEX_PCplus4;
 reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd;
 reg [5:0]  IDEX_func;                            
 reg        IDEX_RegDst, IDEX_ALUSrc;
 reg [1:0]  IDEX_ALUcntrl;
 reg        IDEX_Branch, IDEX_MemRead, IDEX_MemWrite; 
 reg        IDEX_MemToReg, IDEX_RegWrite;                
 reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd; 
 reg [31:0] EXMEM_ALUOut;
 reg        EXMEM_Zero;
 reg [31:0] EXMEM_MemWriteData, EXMEM_rdtemp;
 reg        EXMEM_Branch, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
 reg [31:0] MEMWB_DMemOut;
 reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd; 
 reg [31:0] MEMWB_ALUOut;
 reg        MEMWB_MemToReg, MEMWB_RegWrite; 
 reg [31:0] ALUInA, ALUInB;
 wire [1:0] bypassA, bypassB;            
 wire [31:0] instr, ALUOut, rdA, rdB, signExtend, DMemOut, wRegData, PCIncr;
 wire Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Branch, Jump ;
 reg RegWrite;
 wire [5:0] opcode, func;
 wire [4:0] instr_rs, instr_rt, instr_rd, RegWriteAddr ,shamt;
 wire [3:0] ALUOp;
 wire [1:0] ALUcntrl;
 wire [15:0] imm;
 wire [31:0] rdtemp , EXMEM_rdtemp ;
 wire [27:0] jump_addr;
wire PCWrite, IFIDwrite, bubble_idex ;
 


 
 

/***************** Instruction Fetch Unit (IF)  ****************/
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
       PC <= -1;     
    else if (PC == -1 && PCWrite)
       PC <= 0;
    else if (PCWrite)
	   PC <= PC + 4;
    else if (Jump == 1'b1) 
	PC <= PC | jump_addr ;
end
  
  // IFID pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       IFID_PCplus4 <= 32'b0;    
       IFID_instr <= 32'b0;
    end 
    else 
      begin
	if (IFIDwrite)
           begin
	      IFID_PCplus4 <= PC + 32'd4;
	      IFID_instr <= instr;
	   end
  end
  end
  
// Instruction memory 1KB
Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC>>2, 32'b0, instr);
  
  
  
  
  
/***************** Instruction Decode Unit (ID)  ****************/
assign opcode = IFID_instr[31:26];
assign func = IFID_instr[5:0];
assign instr_rs = IFID_instr[25:21];
assign instr_rt = IFID_instr[20:16];
assign instr_rd = IFID_instr[15:11];
assign imm = IFID_instr[15:0];
assign shamt = IFID_instr[10:6];
assign jump_addr = (opcode == 6'b000010) ? IFID_instr[25:0] <<2  : 32'bx;
assign signExtend = (func == 6'b000000) ? {{27{shamt[4]}}, shamt} : {{16{imm[15]}}, imm};


// Register file
RegFile cpu_regs(clock, reset, instr_rs, instr_rt, MEMWB_RegWriteAddr, MEMWB_RegWrite, wRegData, rdA, rdB);

  // IDEX pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)
      begin
       IDEX_rdA <= 32'b0;    
       IDEX_rdB <= 32'b0;
       IDEX_signExtend <= 32'b0;
       IDEX_instr_rd <= 5'b0;
       IDEX_instr_rs <= 5'b0;
       IDEX_instr_rt <= 5'b0;
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b0;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
    end
    else begin 
    	if  (bubble_idex== 1 ) begin
       		IDEX_RegDst <=1'b0;
      		IDEX_ALUcntrl <=2'b0;
       		IDEX_ALUSrc <=1'b0;
       		IDEX_Branch <=1'b0;
       		IDEX_MemRead <= 1'b0;
       		IDEX_MemWrite <= 1'b0;
       		IDEX_MemToReg <= 1'b0;                  
       		IDEX_RegWrite <= 1'b0;
    	end
	else  
      	begin
      	 	IDEX_rdA <= rdA;
       		IDEX_rdB <= rdB;
       		IDEX_signExtend <= signExtend;
       		IDEX_instr_rd <= instr_rd;
       		IDEX_instr_rs <= instr_rs;
       		IDEX_instr_rt <= instr_rt;
       		IDEX_RegDst <= RegDst;
       		IDEX_ALUcntrl <= ALUcntrl;
       		IDEX_ALUSrc <= ALUSrc;
       		IDEX_Branch <= Branch;
       		IDEX_MemRead <= MemRead;
       		IDEX_MemWrite <= MemWrite;
       		IDEX_MemToReg <= MemToReg;                  
       		IDEX_RegWrite <= RegWrite;
		IDEX_PCplus4  <= IFID_PCplus4;
		IDEX_func <= func;

    	end
     end	
  end

// Main Control Unit 
control_main control_main (.RegDst(RegDst),
		  .Branch(Branch),
		  .Jump(Jump),
                  .MemRead(MemRead),
                  .MemWrite(MemWrite),
                  .MemToReg(MemToReg),
                  .ALUSrc(ALUSrc),
                  .RegWrite(RegWrite),
                  .ALUcntrl(ALUcntrl),
                  .opcode(opcode));
                  
// Instantiation of Control Unit that generates stalls goes here


HazardUnit danger_det(  PCWrite , IFIDwrite , bubble_idex ,  instr_rt , instr_rs , IDEX_MemRead ,IDEX_instr_rt );

   assign new_addr =  IDEX_PCplus4 + (IDEX_signExtend << 2) ;
                         
/***************** Execution Unit (EX)  ****************/
 always @ (*) begin 
	
	if( bypassA == 2'b00) 
	   ALUInA <= IDEX_rdA; 
		
	else if (bypassA == 2'b01) 
		ALUInA <= wRegData;

	else if (bypassA == 2'b10) 
		ALUInA <= EXMEM_ALUOut;
	end

assign rdtemp = (bypassB == 2'b10)? EXMEM_ALUOut : ((bypassB == 2'b01) ? wRegData : IDEX_rdB);

always @ (*) begin 

	if( IDEX_ALUSrc == 1'b0) 
	   ALUInB <= rdtemp; 

	else 
	 ALUInB <= IDEX_signExtend;
end



//  ALU
ALU  #(32) cpu_alu(ALUOut, Zero, ALUInA, ALUInB, ALUOp , IDEX_signExtend ,  IDEX_func);

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

 // EXMEM pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       EXMEM_ALUOut <= 32'b0;    
       EXMEM_RegWriteAddr <= 5'b0;
       EXMEM_MemWriteData <= 32'b0;
       EXMEM_Zero <= 1'b0;
       EXMEM_Branch <= 1'b0;
       EXMEM_MemRead <= 1'b0;
       EXMEM_MemWrite <= 1'b0;
       EXMEM_MemToReg <= 1'b0;                  
       EXMEM_RegWrite <= 1'b0;
       EXMEM_rdtemp <= 32'bx;
      end 
    else 
      begin
       EXMEM_ALUOut <= ALUOut;    
       EXMEM_RegWriteAddr <= RegWriteAddr;
       EXMEM_MemWriteData <= IDEX_rdB;
       EXMEM_Zero <= Zero;
       EXMEM_Branch <= IDEX_Branch;
       EXMEM_MemRead <= IDEX_MemRead;
       EXMEM_MemWrite <= IDEX_MemWrite;
       EXMEM_MemToReg <= IDEX_MemToReg;                  
       EXMEM_RegWrite <= IDEX_RegWrite;
       EXMEM_rdtemp <= rdtemp;
      end
  end
  
  // ALU control
  control_alu control_alu(ALUOp, IDEX_ALUcntrl, IDEX_func);
  
   // Instantiation of control logic for Forwarding goes here
  

  control_bypass_ex forward( bypassA,bypassB,
                        IDEX_instr_rs,
                       IDEX_instr_rt,
                      EXMEM_RegWriteAddr,
                       MEMWB_RegWriteAddr,
                       EXMEM_RegWrite,
                       MEMWB_RegWrite);
       
  
  
/***************** Memory Unit (MEM)  ****************/  

// Data memory 1KB
Memory cpu_DMem(clock, reset, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_ALUOut, EXMEM_rdtemp, DMemOut);

// MEMWB pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       MEMWB_DMemOut <= 32'b0;    
       MEMWB_ALUOut <= 32'b0;
       MEMWB_RegWriteAddr <= 5'b0;
       MEMWB_MemToReg <= 1'b0;                  
       MEMWB_RegWrite <= 1'b0;
      end 
    else 
      begin
       MEMWB_DMemOut <= DMemOut;
       MEMWB_ALUOut <= EXMEM_ALUOut;
       MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
       MEMWB_MemToReg <= EXMEM_MemToReg;                  
       MEMWB_RegWrite <= EXMEM_RegWrite;
      end
  end

  
  
  

/***************** WriteBack Unit (WB)  ****************/  
assign wRegData = (MEMWB_MemToReg == 1'b0) ? MEMWB_ALUOut : MEMWB_DMemOut;


endmodule
