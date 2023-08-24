/**
 * lib.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the library of standard components used by the RISC-V processor,
 * which includes both synchronous and combinational components.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

// RISC-V Includes
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "internal_defines.vh"

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;
    import RISCV_UArch::SUPERSCALAR_WAYS;


/*--------------------------------------------------------------------------------------------------------------------
 * Combinational Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Selects on input from INPUTS inputs to output, each of WIDTH bits.
 *
 * Parameters:
 *  - INPUTS    The number of values from which the mux can select.
 *  - WIDTH     The number of bits each value contains.
 *
 * Inputs:
 *  - in        The values from which to select, packed together as a single
 *              bit-vector.
 *  - sel       The value from the inputs to output.
 *
 * Outputs:
 *  - out       The selected output from the inputs.
 **/
module mux
    #(parameter INPUTS=0, WIDTH=0)
    (input  logic [INPUTS-1:0][WIDTH-1:0]   in,
     input  logic [$clog2(INPUTS)-1:0]      sel,
     output logic [WIDTH-1:0]               out);

    assign out = in[sel];

endmodule: mux

module predicted_nextPC(
  input logic [31:0] tagPC_F_WAY_0, tagPC_F_WAY_1, pc_F_WAY_0,pc_F_WAY_1, BTB_nextPC_WAY_0, BTB_nextPC_WAY_1,
  input logic predicted_bcond_F, predicted_WAY_0_F,
  output logic [31:0] predicted_nextPC_F_WAY_0);

  always_comb begin
      if (predicted_WAY_0_F) begin
        if(tagPC_F_WAY_0[31:2] != pc_F_WAY_0[31:2] || !predicted_bcond_F) begin 
          predicted_nextPC_F_WAY_0 = pc_F_WAY_0 + 8; 
        end
        else begin
          predicted_nextPC_F_WAY_0 = BTB_nextPC_WAY_0; 
        end
      end

      else begin
        if(tagPC_F_WAY_1[31:2] != pc_F_WAY_1[31:2] || !predicted_bcond_F) begin 
          predicted_nextPC_F_WAY_0 = pc_F_WAY_0 + 8; 
        end
        else begin
          predicted_nextPC_F_WAY_0 = BTB_nextPC_WAY_1; 
        end
      end 
  end
endmodule: predicted_nextPC

module nextPCmod(
  input logic choose_E,
  input logic [31:0] next_pc_E_WAY_0, predicted_nextPC_F_WAY_0,
  output logic [31:0] next_pc_WAY_0);
 
  always_comb begin
      if(choose_E) begin
        next_pc_WAY_0 = next_pc_E_WAY_0;
      end
      else begin
        next_pc_WAY_0 = predicted_nextPC_F_WAY_0;
      end
    end

endmodule: nextPCmod

module predicted_bcond(
  input logic [1:0] history,
  output logic predicted_bcond_F);

    always_comb begin
      unique case (history[1])
        1'b0: predicted_bcond_F = 0;
        1'b1: predicted_bcond_F = 1;
        default: predicted_bcond_F = 0;
      endcase
    end

endmodule: predicted_bcond

module ctrlSignals(
  input logic stall,
  input ctrl_signals_t received_ctrl_signals_D,
  output ctrl_signals_t ctrl_signals_D); 

    always_comb begin
      /*
      if (flush) begin
         ctrl_signals_D = '{
            useImm: 1'b0,
            rfWrite: 1'b0, 
            mem2RF: 1'b0,
            pc2RF: 1'b0,
            memRead: 1'b0,  
            memWrite: 1'b0, 
            imm_mode: received_ctrl_signals_D.imm_mode,
            alu_op: ALU_DC,
            ldst_mode: LDST_DC,
            pc_source: PC_DC,

            btype: 3'b0,
            syscall: 1'b0,
            illegal_instr: 1'b0
        };
      end
      */
      if(~stall) ctrl_signals_D = received_ctrl_signals_D;
      else begin
        ctrl_signals_D = '{
            useImm: 1'b0,
            rfWrite: 1'b0, 
            mem2RF: 1'b0,
            pc2RF: 1'b0,
            memRead: 1'b0,  
            memWrite: 1'b0, 
            imm_mode: IMM_DC,
            alu_op: ALU_DC,
            ldst_mode: LDST_DC,
            pc_source: PC_DC,

            btype: received_ctrl_signals_D.btype,
            syscall: received_ctrl_signals_D.syscall,
            illegal_instr: 1'b0
        };

      end
    end


endmodule: ctrlSignals

module chooseRS1(
  input ctrl_signals_t ctrl_signals,
  input logic [31:0] instr,
  output logic [4:0] rs1);

    always_comb begin
      if(ctrl_signals.syscall == 1'b1) rs1 = 5'd10;
      else rs1= instr[19:15];
    end

endmodule: chooseRS1


module old_stall_b(
  input logic [4:0] rs1_D_WAY_0, rs2_D_WAY_0, rd_E_WAY_0, rd_M_WAY_0, rd_W_WAY_0, rd_D_WAY_0,
  input logic [4:0] rs1_D_WAY_1, rs2_D_WAY_1, rd_E_WAY_1, rd_M_WAY_1, rd_W_WAY_1,
  input ctrl_signals_t ctrl_signals_D_WAY_0, ctrl_signals_E_WAY_0, 
  input ctrl_signals_t ctrl_signals_M_WAY_0, ctrl_signals_W_WAY_0,
  input ctrl_signals_t ctrl_signals_D_WAY_1, ctrl_signals_E_WAY_1, 
  input ctrl_signals_t ctrl_signals_M_WAY_1, ctrl_signals_W_WAY_1,
  input logic usingRS1_WAY_0, usingRS2_WAY_0, usingRS1_WAY_1, usingRS2_WAY_1,
  input logic [31:0] pc_D_WAY_1, pc_E_WAY_1,
  input logic mem_concur_D, rf_write_way_0_D, prev_stall_case_1, no_op_WAY_0_E, prev_concur,
  output logic stall_WAY_0, stall_WAY_1, stall_F_D_WAY_0, no_op_WAY_0,  
  output logic data_stall, instr_stall, stall_case_1, concur, halt_next);

  //opcode_t opcode;
  //assign opcode = opcode_t'(instr[6:0]);

  logic check_data_hazard, check_1, check_2, check_3, check_4;
  logic check_concur_hazard, check_concur_1, check_concur_2, check_all_rs, check_concur_prev;
  always_comb begin
    check_1 = (
            {ctrl_signals_E_WAY_0.memRead || (ctrl_signals_E_WAY_0.alu_op == ALU_MUL)} &&
            {( ((rs1_D_WAY_0 == rd_E_WAY_0) && usingRS1_WAY_0) && (rs1_D_WAY_0 != 0) ) ||
             ( ((rs2_D_WAY_0 == rd_E_WAY_0) && usingRS2_WAY_0) && (rs1_D_WAY_0 != 0) )} 
            );

    check_2 =  (
             {ctrl_signals_E_WAY_1.memRead || (ctrl_signals_E_WAY_1.alu_op == ALU_MUL)} &&
             {( ((rs1_D_WAY_0 == rd_E_WAY_1) && usingRS1_WAY_0) && (rs1_D_WAY_0 != 0) ) ||
              ( ((rs2_D_WAY_0 == rd_E_WAY_1) && usingRS2_WAY_0) && (rs1_D_WAY_0 != 0) )}
            );

    check_3 = (
             {ctrl_signals_E_WAY_0.memRead || (ctrl_signals_E_WAY_0.alu_op == ALU_MUL)} &&
             {( ((rs1_D_WAY_1 == rd_E_WAY_0) && usingRS1_WAY_1) && (rs1_D_WAY_1 != 0) ) ||
              ( ((rs2_D_WAY_1 == rd_E_WAY_0) && usingRS2_WAY_1) && (rs1_D_WAY_1 != 0) ) }
            );
    check_4 = (
             {ctrl_signals_E_WAY_1.memRead || (ctrl_signals_E_WAY_1.alu_op == ALU_MUL)} &&
             {( ((rs1_D_WAY_1 == rd_E_WAY_1) && usingRS1_WAY_1) && (rs1_D_WAY_1 != 0) ) ||
              ( ((rs2_D_WAY_1 == rd_E_WAY_1) && usingRS2_WAY_1) && (rs1_D_WAY_1 != 0) )}
            );

    check_data_hazard = (check_1 || check_2 || check_3 || check_4);
    check_concur_1 = (rs1_D_WAY_1 == rd_D_WAY_0) && usingRS1_WAY_1;
    check_concur_2 = (rs2_D_WAY_1 == rd_D_WAY_0) && usingRS2_WAY_1;
    check_all_rs = check_concur_1 || check_concur_2;
    check_concur_prev = (pc_D_WAY_1 != pc_E_WAY_1) || prev_stall_case_1;

    check_concur_hazard = (check_all_rs && rf_write_way_0_D && (~no_op_WAY_0_E)) && check_concur_prev;
   end

  
  always_comb begin
           //D stage rfWrite, W_stage same instr but NO_OP, need case 1 to proc
          if (check_data_hazard && check_concur_hazard)

            begin
             stall_WAY_0 = 1;
             stall_F_D_WAY_0 = 1;
             stall_WAY_1 = 1;
             instr_stall = 1;
             data_stall = 1;
             no_op_WAY_0 = 0;
             stall_case_1 = 1;
             concur = 0;
              halt_next=0;
            end

          else if (check_data_hazard && prev_concur)
            begin
              stall_WAY_0 = 1;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 1;
              stall_case_1 = 1;
              concur = 0;
              halt_next=0;
            end

          else if (check_data_hazard && (~prev_concur))
            begin
              stall_WAY_0 = 1;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 0;
              stall_case_1 = 1;
              concur = 0;
              halt_next=0;
            end

          //case 1: dependency betwen concurrent ways
          else if (check_concur_hazard)
            begin
              stall_WAY_0 = 0;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 1;
              stall_case_1 = 0;
              concur = 1;
              halt_next=0;
            end 

          else if (mem_concur_D && ((pc_D_WAY_1 != pc_E_WAY_1) || (prev_stall_case_1)))
            begin
              stall_WAY_0 = 0;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 1;
              stall_case_1 = 0;
              concur = 1;
              halt_next=0;
            end
         

          //case 5: syscall at way0
          else if
           ((ctrl_signals_D_WAY_0.syscall || ctrl_signals_E_WAY_0.syscall || ctrl_signals_M_WAY_0.syscall) 
            && (~ctrl_signals_W_WAY_0.syscall))
            begin
              stall_WAY_0 = 1;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 0;
              stall_case_1 = 0;
              concur = 0;
              halt_next=0;
            end

          //case 6: syscall at way1
          else if 
           ((ctrl_signals_D_WAY_1.syscall || ctrl_signals_E_WAY_1.syscall || ctrl_signals_M_WAY_1.syscall) 
            && (~ctrl_signals_W_WAY_1.syscall))
            begin
              stall_WAY_0 = 0;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 1;
              stall_case_1 = 0;
              concur = 0;
              halt_next=0;
            end

          else if 
           (ctrl_signals_D_WAY_1.syscall && ctrl_signals_E_WAY_1.syscall && ctrl_signals_M_WAY_1.syscall 
            && ctrl_signals_W_WAY_1.syscall)
            begin
              stall_WAY_0 = 0;
              stall_F_D_WAY_0 = 1;
              stall_WAY_1 = 1;
              instr_stall = 1;
              data_stall = 1;
              no_op_WAY_0 = 1;
              stall_case_1 = 0;
              concur = 0;
              halt_next = 1;
            end
          
          else begin
              stall_WAY_0 = 0;
              stall_F_D_WAY_0 = 0;
              stall_WAY_1 = 0;
              instr_stall = 0;
              data_stall = 0;
              no_op_WAY_0 = 0;
              stall_case_1 = 0;
              concur = 0;
              halt_next=0;
          end
    end


endmodule: old_stall_b

/*
module stall_b
    (input  logic [4:0]  rs1_D,
     input  logic [4:0]  rd_E,
     input  logic [4:0]  rd_M,
     input  logic [4:0]  rd_W,
     input  logic [4:0]  rs2_D,
     input  logic ctrl_signals_D_syscall,
     input  logic ctrl_signals_E_memRead,
     input  logic ctrl_signals_E_syscall,
     input  logic ctrl_signals_M_syscall,
     input  logic ctrl_signals_W_syscall,
     input  logic [3:0] ctrl_signals_E_alu_op,
     input  logic ctrl_signals_E_rfWrite,
     input  logic ctrl_signals_M_rfWrite,
     input  logic ctrl_signals_W_rfWrite,   
     input  logic usingRS1,
     input  logic usingRS2,
     output logic data_stall,
     output logic instr_stall);

    always_comb begin
      if(   {{ctrl_signals_E_memRead || (ctrl_signals_E_alu_op == ALU_MUL)} &&
            {( ((rs1_D == rd_E) && usingRS1) && (rs1_D != 0) ) ||
            (((rs2_D == rd_E) && usingRS2) && (rs1_D != 0) )}}
            || 
            
         // {ctrl_signals_D_syscall && 
         //{((rs1_D == rd_E) && ctrl_signals_E_rfWrite && usingRS1) ||
         //((rs1_D == rd_M) && ctrl_signals_M_rfWrite && usingRS1) ||
         //((rs1_D == rd_W) && ctrl_signals_W_rfWrite && usingRS1)}} ||
           
         //  {(!ctrl_signals_W_syscall) && 
         //  {ctrl_signals_D_syscall || ctrl_signals_E_syscall || ctrl_signals_M_syscall}} )
      begin
          data_stall = 1'b1; instr_stall = 1'b1;
      end
      else begin data_stall = 1'b0; instr_stall = 1'b0; end   
    end

endmodule: stall_b
*/

/**
 * Adds two numbers of WIDTH bits, with a carry in bit, producing a sum and a
 * carry out bit.
 *
 * Parameters:
 *  - WIDTH     The number of bits of the numbers being summed together.
 *
 * Inputs:
 *  - cin       The carry in to the addition.
 *  - A         The first number to add.
 *  - B         The second number to add.no_op_WAY_0
 *
 * Outputs:
 *  - cout      The carry out from the addition.
 *  - sum       The result of the addition.
 **/
module adder
    #(parameter WIDTH=0)
    (input  logic               cin,
     input  logic [WIDTH-1:0]   A, B,
     output logic               cout,
     output logic [WIDTH-1:0]   sum);

     assign {cout, sum} = A + B + cin;

endmodule: adder

/*--------------------------------------------------------------------------------------------------------------------
 * Synchronous Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Latches and stores values of WIDTH bits and initializes to RESET_VAL.
 *
 * This register uses an asynchronous active-low reset and a synchronous
 * active-high clear. Upon clear or reset, the value of the register becomes
 * RESET_VAL.
 *
 * Parameters:
 *  - WIDTH         The number of bits that the register holds.
 *  - RESET_VAL     The value that the register holds after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the register.
 *  - rst_l         An active-low asynchronous reset.
 *  - clear         An active-high synchronous reset.
 *  - en            Indicates whether or not to load the register.
 *  - D             The input to the register.
 *
 * Outputs:
 *  - Q             The latched output from the register.
 **/
module register
   #(parameter                      WIDTH=0,
     parameter logic [WIDTH-1:0]    RESET_VAL='b0)
    (input  logic               clk, en, rst_l, clear,
     input  logic [WIDTH-1:0]   D,
     output logic [WIDTH-1:0]   Q);

     always_ff @(posedge clk, negedge rst_l) begin
         if (!rst_l)
             Q <= RESET_VAL;
         else if (clear)
             Q <= RESET_VAL;
         else if (en)
             Q <= D;
     end

endmodule:register




//possibly could speed up crit path if we had this separated into 4 modules
module forward(
  input logic forward_rs1_E_E_WAY_0_0, forward_rs1_M_E_WAY_0_0, forward_rs2_E_E_WAY_0_0, forward_rs2_M_E_WAY_0_0,
  input logic forward_rs1_E_E_WAY_0_1, forward_rs1_M_E_WAY_0_1, forward_rs2_E_E_WAY_0_1, forward_rs2_M_E_WAY_0_1,
  input logic forward_rs1_E_E_WAY_1_0, forward_rs1_M_E_WAY_1_0, forward_rs2_E_E_WAY_1_0, forward_rs2_M_E_WAY_1_0,
  input logic forward_rs1_E_E_WAY_1_1, forward_rs1_M_E_WAY_1_1, forward_rs2_E_E_WAY_1_1, forward_rs2_M_E_WAY_1_1,
  input ctrl_signals_t ctrl_signals_W_WAY_0, ctrl_signals_W_WAY_1,
  input ctrl_signals_t ctrl_signals_M_WAY_0, ctrl_signals_M_WAY_1,
  input ctrl_signals_t ctrl_signals_E_WAY_0, ctrl_signals_E_WAY_1,
   
  input logic [31:0] mult_out_W_WAY_0, mult_out_W_WAY_1,
  input logic [31:0] rd_data_W_WAY_0, rd_data_W_WAY_1,
  
  input logic [31:0] rs2_data_E_WAY_0, rs2_data_E_WAY_1,
  input logic [31:0] rs1_data_E_WAY_0, rs1_data_E_WAY_1,
  input logic [31:0] alu_out_M_WAY_0, alu_out_M_WAY_1,

  output logic [31:0] rs1_data_final_E_WAY_0, rs1_data_final_E_WAY_1, 
  output logic [31:0] rs2_data_final_E_WAY_0, rs2_data_final_E_WAY_1);

  //forward rs1 logic block WAY 0
  always_comb begin
    if(forward_rs1_E_E_WAY_1_0 && (!ctrl_signals_E_WAY_0.syscall) && (ctrl_signals_M_WAY_1.memRead != 1'b1))
      begin
        rs1_data_final_E_WAY_0 = alu_out_M_WAY_1;
      end

    else if(forward_rs1_E_E_WAY_0_0 && (!ctrl_signals_E_WAY_0.syscall) && (ctrl_signals_M_WAY_0.memRead != 1'b1))
      begin
        rs1_data_final_E_WAY_0 = alu_out_M_WAY_0;
      end

    else if(forward_rs1_M_E_WAY_1_0 && (!ctrl_signals_E_WAY_0.syscall))
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs1_data_final_E_WAY_0 = mult_out_W_WAY_1;
        else rs1_data_final_E_WAY_0 = rd_data_W_WAY_1;
      end

    else if(forward_rs1_M_E_WAY_0_0 && (!ctrl_signals_E_WAY_0.syscall))
      begin
        if(ctrl_signals_W_WAY_0.alu_op == ALU_MUL) rs1_data_final_E_WAY_0 = mult_out_W_WAY_0;
        else rs1_data_final_E_WAY_0 = rd_data_W_WAY_0;
      end

    else rs1_data_final_E_WAY_0 = rs1_data_E_WAY_0;

  end

  //forward rs1 logic block WAY 1
  always_comb begin
    if(forward_rs1_E_E_WAY_1_1 && (!ctrl_signals_E_WAY_1.syscall) && (ctrl_signals_M_WAY_1.memRead != 1'b1))
      begin
        rs1_data_final_E_WAY_1 = alu_out_M_WAY_1;
      end

    else if(forward_rs1_E_E_WAY_0_1 && (!ctrl_signals_E_WAY_1.syscall) && (ctrl_signals_M_WAY_0.memRead != 1'b1))
      begin
        rs1_data_final_E_WAY_1 = alu_out_M_WAY_0;
      end

    else if(forward_rs1_M_E_WAY_1_1 && (!ctrl_signals_E_WAY_1.syscall))
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs1_data_final_E_WAY_1 = mult_out_W_WAY_1;
        else rs1_data_final_E_WAY_1 = rd_data_W_WAY_1;
      end

    else if(forward_rs1_M_E_WAY_0_1 && (!ctrl_signals_E_WAY_1.syscall))
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs1_data_final_E_WAY_1 = mult_out_W_WAY_0;
        else rs1_data_final_E_WAY_1 = rd_data_W_WAY_0;
      end

    else rs1_data_final_E_WAY_1 = rs1_data_E_WAY_1;

  end

  //forward rs2 logic block WAY 0
  always_comb begin
    if(forward_rs2_E_E_WAY_1_0 && (!ctrl_signals_E_WAY_0.syscall) && (ctrl_signals_M_WAY_1.memRead != 1'b1))
      begin
        rs2_data_final_E_WAY_0 = alu_out_M_WAY_1;
      end

    else if(forward_rs2_E_E_WAY_0_0 && (!ctrl_signals_E_WAY_0.syscall) && (ctrl_signals_M_WAY_0.memRead != 1'b1))
      begin
        rs2_data_final_E_WAY_0 = alu_out_M_WAY_0;
      end

    else if(forward_rs2_M_E_WAY_1_0 && (!ctrl_signals_E_WAY_0.syscall))
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs2_data_final_E_WAY_0 = mult_out_W_WAY_1;
        else rs2_data_final_E_WAY_0 = rd_data_W_WAY_1;
      end

    else if(forward_rs2_M_E_WAY_0_0 && (!ctrl_signals_E_WAY_0.syscall))
      begin
        if(ctrl_signals_W_WAY_0.alu_op == ALU_MUL) rs2_data_final_E_WAY_0 = mult_out_W_WAY_0;
        else rs2_data_final_E_WAY_0 = rd_data_W_WAY_0;
      end

    else rs2_data_final_E_WAY_0 = rs2_data_E_WAY_0;

  end

  //forward rs2 logic block WAY 1
  always_comb begin
    if(forward_rs2_E_E_WAY_1_1 && (ctrl_signals_M_WAY_1.memRead != 1'b1))
      begin
        rs2_data_final_E_WAY_1 = alu_out_M_WAY_1;
      end

    else if(forward_rs2_E_E_WAY_0_1 && (ctrl_signals_M_WAY_0.memRead != 1'b1))
      begin
        rs2_data_final_E_WAY_1 = alu_out_M_WAY_0;
      end

    else if(forward_rs2_M_E_WAY_1_1)
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs2_data_final_E_WAY_1 = mult_out_W_WAY_1;
        else rs2_data_final_E_WAY_1 = rd_data_W_WAY_1;
      end

    else if(forward_rs2_M_E_WAY_0_1)
      begin
        if(ctrl_signals_W_WAY_1.alu_op == ALU_MUL) rs2_data_final_E_WAY_1 = mult_out_W_WAY_0;
        else rs2_data_final_E_WAY_1 = rd_data_W_WAY_0;
      end

    else rs2_data_final_E_WAY_1 = rs2_data_E_WAY_1;

  end
endmodule: forward


module forward_en(
  input  logic [4:0] rs_D_WAY_0, rd_E_WAY_0, rd_M_WAY_0,
  input  logic [4:0] rs_D_WAY_1, rd_E_WAY_1, rd_M_WAY_1, 
  input ctrl_signals_t ctrl_signals_E_WAY_0, ctrl_signals_M_WAY_0,
  input ctrl_signals_t ctrl_signals_E_WAY_1, ctrl_signals_M_WAY_1,
  input  logic flush_M,  
  output logic forward_rs_E_WAY_0_0, forward_rs_M_WAY_0_0,
  output logic forward_rs_E_WAY_0_1, forward_rs_M_WAY_0_1,
  output logic forward_rs_E_WAY_1_0, forward_rs_M_WAY_1_0,
  output logic forward_rs_E_WAY_1_1, forward_rs_M_WAY_1_1);

  //WAY 0 
  always_comb begin

   if ( (~flush_M) && (rs_D_WAY_0 != 0) && (rs_D_WAY_0 == rd_E_WAY_1) && ctrl_signals_E_WAY_1.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_0 = 0;forward_rs_M_WAY_0_0 = 0;
        forward_rs_E_WAY_1_0 = 1;forward_rs_M_WAY_1_0 = 0;
   end
   
   else if ( (rs_D_WAY_0 != 0) && (rs_D_WAY_0 == rd_E_WAY_0) && ctrl_signals_E_WAY_0.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_0 = 1;forward_rs_M_WAY_0_0 = 0;
        forward_rs_E_WAY_1_0 = 0;forward_rs_M_WAY_1_0 = 0;
   end

   else if ( (rs_D_WAY_0 != 0) && (rs_D_WAY_0 == rd_M_WAY_1) && ctrl_signals_M_WAY_1.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_0 = 0;forward_rs_M_WAY_0_0 = 0;
        forward_rs_E_WAY_1_0 = 0;forward_rs_M_WAY_1_0 = 1;
   end

   else if ( (rs_D_WAY_0 != 0) && (rs_D_WAY_0 == rd_M_WAY_0) && ctrl_signals_M_WAY_0.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_0 = 0;forward_rs_M_WAY_0_0 = 1;
        forward_rs_E_WAY_1_0 = 0;forward_rs_M_WAY_1_0 = 0;
   end

   else begin
        forward_rs_E_WAY_0_0 = 0;forward_rs_M_WAY_0_0 = 0;
        forward_rs_E_WAY_1_0 = 0;forward_rs_M_WAY_1_0 = 0;
   end
  end

  


  //WAY 1
  always_comb begin
      if ((~flush_M) && (rs_D_WAY_1 != 0) && (rs_D_WAY_1 == rd_E_WAY_1) && ctrl_signals_E_WAY_1.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_1 = 0;forward_rs_M_WAY_0_1 = 0;
        forward_rs_E_WAY_1_1 = 1;forward_rs_M_WAY_1_1 = 0;
      end

      else if ((rs_D_WAY_1 != 0) && (rs_D_WAY_1 == rd_E_WAY_0) && ctrl_signals_E_WAY_0.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_1 = 1;forward_rs_M_WAY_0_1 = 0;
        forward_rs_E_WAY_1_1 = 0;forward_rs_M_WAY_1_1 = 0;
      end     

      else if ( (rs_D_WAY_1 != 0) && (rs_D_WAY_1 == rd_M_WAY_1) && ctrl_signals_M_WAY_1.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_1 = 0;forward_rs_M_WAY_0_1 = 0;
        forward_rs_E_WAY_1_1 = 0;forward_rs_M_WAY_1_1 = 1;
      end

      else if ( (rs_D_WAY_1 != 0) && (rs_D_WAY_1 == rd_M_WAY_0) && ctrl_signals_M_WAY_0.rfWrite) begin
        //forward writeback value from MEM
        forward_rs_E_WAY_0_1 = 0;forward_rs_M_WAY_0_1 = 1;
        forward_rs_E_WAY_1_1 = 0;forward_rs_M_WAY_1_1 = 0;
      end

      else begin
        //use original D stage values, no forwarding
        forward_rs_E_WAY_0_1 = 0;forward_rs_M_WAY_0_1 = 0;
        forward_rs_E_WAY_1_1 = 0;forward_rs_M_WAY_1_1 = 0;
      end
    end

endmodule: forward_en

module dataStore(
  input ctrl_signals_t ctrl_signals_E_WAY_0, ctrl_signals_E_WAY_1,
  input logic ldstr_E_WAY_0,
  input logic [31:0] rs2_data_final_E_WAY_0, rs2_data_final_E_WAY_1, alu_out_E_WAY_0, alu_out_E_WAY_1,  
  output logic [3:0] data_store_mask_E,
  output logic [31:0] data_store_E);

  logic [31:0] store_shift_E;

  

  always_comb begin
      begin
        if(ldstr_E_WAY_0) begin
          store_shift_E = {30'b0,alu_out_E_WAY_0[1:0]};
          if(ctrl_signals_E_WAY_0.imm_mode == IMM_S && (ctrl_signals_E_WAY_0.memWrite == 1'b1)) begin
            unique case(ctrl_signals_E_WAY_0.ldst_mode)
              LDST_W: begin
               data_store_mask_E = 4'b1111;
               data_store_E = rs2_data_final_E_WAY_0;
               end

              LDST_B: begin
                data_store_mask_E = 4'b0001 << store_shift_E;
                data_store_E = rs2_data_final_E_WAY_0 << (store_shift_E << 3);
              end
      
              LDST_H: begin
                data_store_mask_E = 4'b0011 << store_shift_E;
                data_store_E = rs2_data_final_E_WAY_0 << (store_shift_E << 3);
              end

              default: begin
                 data_store_mask_E = 4'b0;
                 data_store_E = rs2_data_final_E_WAY_0;
                       end
            endcase
          end
      
          else begin 
            data_store_mask_E = 4'b0;
            data_store_E = rs2_data_final_E_WAY_0;
          end
        end

        else begin
          store_shift_E = {30'b0,alu_out_E_WAY_1[1:0]};
          if(ctrl_signals_E_WAY_1.imm_mode == IMM_S && (ctrl_signals_E_WAY_1.memWrite == 1'b1)) begin
            unique case(ctrl_signals_E_WAY_1.ldst_mode)
              LDST_W: begin
               data_store_mask_E = 4'b1111;
               data_store_E = rs2_data_final_E_WAY_1;
               end

              LDST_B: begin
                data_store_mask_E = 4'b0001 << store_shift_E;
                data_store_E = rs2_data_final_E_WAY_1 << (store_shift_E << 3);
              end
      
              LDST_H: begin
                data_store_mask_E = 4'b0011 << store_shift_E;
                data_store_E = rs2_data_final_E_WAY_1 << (store_shift_E << 3);
              end

              default: begin
                 data_store_mask_E = 4'b0;
                 data_store_E = rs2_data_final_E_WAY_1;
                       end
            endcase
          end
      
          else begin 
            data_store_mask_E = 4'b0;
            data_store_E = rs2_data_final_E_WAY_1;
          end

        end
      end
    end

endmodule: dataStore


module ldstr_M_signals(
  input logic ldstr_M_WAY_0,
  input logic [31:0] alu_out_M_WAY_0, alu_out_M_WAY_1,
  input ctrl_signals_t ctrl_signals_M_WAY_0, ctrl_signals_M_WAY_1,
  output logic data_load_en,
  output logic [29:0] data_addr,
  output logic [31:0] store_shift, data_offset_M);

    always_comb begin
      if(ldstr_M_WAY_0) begin 
        data_addr  = alu_out_M_WAY_0[31:2];
        data_load_en = ctrl_signals_M_WAY_0.memRead;
        store_shift = {30'b0,alu_out_M_WAY_0[1:0]};
        data_offset_M = store_shift << 3;
      end
      else begin 
        data_addr = alu_out_M_WAY_1[31:2];
        data_load_en = ctrl_signals_M_WAY_1.memRead;
        store_shift = {30'b0,alu_out_M_WAY_1[1:0]};
        data_offset_M = store_shift << 3;
      end
    end
endmodule: ldstr_M_signals



module dataLoad(
  input ctrl_signals_t ctrl_signals_M_WAY_0, ctrl_signals_M_WAY_1,
  input logic data_load_en_M, ldstr_M_WAY_0,
  input logic [31:0] data_load_M, data_shifted_M,
  output logic [31:0] data_res_M);

  logic [7:0] data_byte_M;
  logic [15:0] data_half_M;

  always_comb begin
    if(ldstr_M_WAY_0) begin
      if(data_load_en_M) begin
        unique case(ctrl_signals_M_WAY_0.ldst_mode)
          LDST_W: begin
            data_res_M = data_load_M;
            data_byte_M = 32'b0;
          end
          
          LDST_B: begin 
            data_byte_M = data_shifted_M[7:0];
            data_res_M = {{25{data_byte_M[7]}},data_byte_M[6:0]};
          end

          LDST_BU: begin 
            data_byte_M = data_shifted_M[7:0];
            data_res_M = {{24{1'b0}}, data_byte_M[7:0]};
          end

          LDST_H: begin 
            data_half_M = data_shifted_M[15:0];
            data_res_M = {{17{data_half_M[15]}},data_half_M[14:0]};
          end

          LDST_HU: begin 
            data_half_M = data_shifted_M[15:0];
            data_res_M = {{16{1'b0}},data_half_M[15:0]};
          end

          default: begin
                     data_res_M = data_load_M;
                     data_byte_M = 8'b0;
                     data_half_M = 16'b0;
                   end
        endcase
      end
      else data_res_M = data_load_M;
    end

    else begin
      if(data_load_en_M) begin
        unique case(ctrl_signals_M_WAY_1.ldst_mode)
          LDST_W: begin
            data_res_M = data_load_M;
            data_byte_M = 32'b0;
          end
          
          LDST_B: begin 
            data_byte_M = data_shifted_M[7:0];
            data_res_M = {{25{data_byte_M[7]}},data_byte_M[6:0]};
          end

          LDST_BU: begin 
            data_byte_M = data_shifted_M[7:0];
            data_res_M = {{24{1'b0}}, data_byte_M[7:0]};
          end

          LDST_H: begin 
            data_half_M = data_shifted_M[15:0];
            data_res_M = {{17{data_half_M[15]}},data_half_M[14:0]};
          end

          LDST_HU: begin 
            data_half_M = data_shifted_M[15:0];
            data_res_M = {{16{1'b0}},data_half_M[15:0]};
          end

          default: begin
                     data_res_M = data_load_M;
                     data_byte_M = 32'b0;
                   end
        endcase
      end
      else data_res_M = data_load_M;
    end
  end

endmodule: dataLoad

module rd_dataLogic(
  input ctrl_signals_t ctrl_signals_M,
  input logic [31:0] data_res_M, mult_out_M, alu_out_M, pc_M,
  output logic [31:0] rd_data_M);

  always_comb begin
      if(ctrl_signals_M.pc_source == PC_uncond) rd_data_M = pc_M + 4;
      else if (ctrl_signals_M.alu_op == ALU_MUL) rd_data_M = mult_out_M;
      else rd_data_M = ctrl_signals_M.mem2RF?data_res_M:alu_out_M;
    end

endmodule: rd_dataLogic


module branchP(
   input ctrl_signals_t ctrl_signals_E_WAY_0, ctrl_signals_E_WAY_1,
   input logic predicted_bcond_E,
   input logic bcond_WAY_0, bcond_WAY_1,
   input logic [31:0] predicted_nextPC_E,
   input logic [31:0] alu_out_E_WAY_0, alu_out_E_WAY_1,
   input logic [31:0] npc_offset_E_WAY_0, npc_offset_E_WAY_1,
   input logic [31:0] pc_E_WAY_0, pc_E_WAY_1,
   input logic predicted_WAY_0_E,
   output logic [31:0] next_pc_E_WAY_0,
   output logic choose_E,
   output logic flush, flush_M,
   output logic BTB_write_WAY_0, BTB_write_WAY_1
   );
   always_comb begin
      if(ctrl_signals_E_WAY_0.pc_source == PC_uncond && 
         ((!predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != alu_out_E_WAY_0)) 
        ) 
        begin
          next_pc_E_WAY_0 = alu_out_E_WAY_0;
          flush = 1'b1;
          choose_E = 1;
          flush_M = 1'b1;
          BTB_write_WAY_0 = 1;
          BTB_write_WAY_1 = 0;
        end

      else if (
            (ctrl_signals_E_WAY_0.pc_source == PC_cond) && bcond_WAY_0 
            && {(!predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != npc_offset_E_WAY_0)}
            ) 
        begin
           next_pc_E_WAY_0 = npc_offset_E_WAY_0;
           flush = 1'b1;
           choose_E = 1;
           flush_M = 1'b1;
           BTB_write_WAY_0 = 1;
           BTB_write_WAY_1 = 0;
        end
      
      else if (
            (ctrl_signals_E_WAY_0.pc_source == PC_cond) && (!bcond_WAY_0)
            && {(ctrl_signals_E_WAY_1.pc_source != PC_cond) && 
                (ctrl_signals_E_WAY_1.pc_source != PC_uncond)}
            && {(!predicted_WAY_0_E) || (predicted_bcond_E) || (predicted_nextPC_E != pc_E_WAY_0 + 8)}
      )
        begin
           next_pc_E_WAY_0 = pc_E_WAY_0 + 8;
           flush = 1'b1;
           choose_E = 1;
           flush_M = 1'b0;
           BTB_write_WAY_0 = 1;
           BTB_write_WAY_1 = 0;
        end

      else if(ctrl_signals_E_WAY_0.pc_source == PC_uncond && predicted_WAY_0_E  && predicted_bcond_E) 
       begin
          next_pc_E_WAY_0 = 32'b0;
          flush = 1'b0;
          choose_E = 0;
          flush_M = 1'b1;
          BTB_write_WAY_0 = 0;
          BTB_write_WAY_1 = 0;
       end
      
      else if(predicted_WAY_0_E && predicted_bcond_E && bcond_WAY_0 && (predicted_nextPC_E == npc_offset_E_WAY_0)) 
       begin
          next_pc_E_WAY_0 = 32'b0;
          flush = 1'b0;
          choose_E = 0;
          flush_M = 1'b1;
          BTB_write_WAY_0 = 0;
          BTB_write_WAY_1 = 0;
       end

      else if((ctrl_signals_E_WAY_1.pc_source == PC_uncond) && (ctrl_signals_E_WAY_0.pc_source == PC_cond) &&
              ((predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != alu_out_E_WAY_1))
             ) 
        begin
          next_pc_E_WAY_0 = alu_out_E_WAY_1;
          flush = 1'b1;
          choose_E = 1;
          flush_M = 1'b0;
          BTB_write_WAY_0 = 1;
          BTB_write_WAY_1 = 1;
        end

      else if((ctrl_signals_E_WAY_1.pc_source == PC_uncond) && (ctrl_signals_E_WAY_0.pc_source != PC_cond) &&
              ((predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != alu_out_E_WAY_1))
             ) 
        begin
          next_pc_E_WAY_0 = alu_out_E_WAY_1;
          flush = 1'b1;
          choose_E = 1;
          flush_M = 1'b0;
          BTB_write_WAY_0 = 0;
          BTB_write_WAY_1 = 1;
        end

      else if ((ctrl_signals_E_WAY_1.pc_source == PC_cond) && bcond_WAY_1 && (ctrl_signals_E_WAY_0.pc_source == PC_cond) &&
               {(predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != npc_offset_E_WAY_1)}) 
        begin
           next_pc_E_WAY_0 = npc_offset_E_WAY_1;
           flush = 1'b1;
           choose_E = 1;
           flush_M = 1'b0;
           BTB_write_WAY_0 = 1;
           BTB_write_WAY_1 = 1;
        end

      else if ((ctrl_signals_E_WAY_1.pc_source == PC_cond) && bcond_WAY_1 && (ctrl_signals_E_WAY_0.pc_source != PC_cond) &&
               {(predicted_WAY_0_E) || (!predicted_bcond_E) || (predicted_nextPC_E != npc_offset_E_WAY_1)}) 
        begin
           next_pc_E_WAY_0 = npc_offset_E_WAY_1;
           flush = 1'b1;
           choose_E = 1;
           flush_M = 1'b0;
           BTB_write_WAY_0 = 0;
           BTB_write_WAY_1 = 1;
        end

      else if ((ctrl_signals_E_WAY_1.pc_source == PC_cond) && (!bcond_WAY_1) && (ctrl_signals_E_WAY_0.pc_source == PC_cond) &&
               {(predicted_WAY_0_E) || (predicted_bcond_E) || (predicted_nextPC_E != pc_E_WAY_0 + 8)})
        begin
          next_pc_E_WAY_0 = pc_E_WAY_0 + 8;
          flush = 1'b1;
          choose_E = 1;
          flush_M = 1'b0;
          BTB_write_WAY_0 = 1;
          BTB_write_WAY_1 = 1;
        end

      else if ((ctrl_signals_E_WAY_1.pc_source == PC_cond) && (!bcond_WAY_1) && (ctrl_signals_E_WAY_0.pc_source != PC_cond) &&
               {(predicted_WAY_0_E) || (predicted_bcond_E) || (predicted_nextPC_E != pc_E_WAY_0 + 8)})
        begin
          next_pc_E_WAY_0 = pc_E_WAY_0 + 8;
          flush = 1'b1;
          choose_E = 1;
          flush_M = 1'b0;
          BTB_write_WAY_0 = 0;
          BTB_write_WAY_1 = 1;
        end
      
      else begin
        next_pc_E_WAY_0 = 32'b0;
        choose_E = 0; 
        flush = 0;
        flush_M = 1'b0;
        BTB_write_WAY_0 = 0;
        BTB_write_WAY_1 = 0;
      end
    end

endmodule: branchP

module update(
  input logic hysteresis_en,
  input logic [1:0] history_E,
  input logic taken,
  output logic [1:0] updated_history
             );

    always_comb begin
      if (hysteresis_en) begin
         unique case (history_E)
           2'b00: begin
              if (taken) updated_history = 2'b10;
              else updated_history = 2'b01;
           end

           2'b01: begin
              if (taken) updated_history = 2'b00;
              else updated_history = 2'b01;
           end

           2'b11: begin
              if (taken) updated_history = 2'b10;
              else updated_history = 2'b01;
           end

           2'b10: begin
              if (taken) updated_history = 2'b10;
              else updated_history = 2'b11;
           end

           default: begin
              updated_history = 2'b00;
           end
        endcase
      end
      else begin
        updated_history = 2'b00;
      end
    end

endmodule: update

module mem_concurrency(
  input logic [31:0] instr_F_WAY_0, instr_F_WAY_1,
  output logic mem_concur);

  opcode_t opcode_WAY_0, opcode_WAY_1;
  assign opcode_WAY_0 = opcode_t'(instr_F_WAY_0[6:0]);
  assign opcode_WAY_1 = opcode_t'(instr_F_WAY_1[6:0]);

  always_comb begin
    if((opcode_WAY_0 == OP_LOAD || opcode_WAY_0 == OP_STORE) &&
      (opcode_WAY_1 == OP_LOAD || opcode_WAY_1 == OP_STORE))
      begin
        mem_concur = 1'b1;
      end

    else mem_concur = 1'b0;
  end


endmodule: mem_concurrency


module check_rfWrite_mod(
  input logic [31:0] instr_F_WAY_0,
  output logic rf_write_way_0);

  opcode_t opcode_WAY_0;
  assign opcode_WAY_0 = opcode_t'(instr_F_WAY_0[6:0]);

  always_comb begin
    if ((opcode_WAY_0 != OP_STORE) && 
        (opcode_WAY_0 != OP_SYSTEM) &&
        (opcode_WAY_0 != OP_BRANCH))
      begin
        rf_write_way_0 = 1'b1;
      end

    else rf_write_way_0 = 1'b0;
  end


endmodule: check_rfWrite_mod

module check_F_control_flow(
   input logic [31:0] instr_F_WAY_0,
   input logic [31:0] instr_F_WAY_1,
   input logic predicted_bcond_F_WAY_0, predicted_bcond_F_WAY_1,
   output logic predicted_WAY_0_F, predicted_bcond_F);

   opcode_t opcode_WAY_0;
   opcode_t opcode_WAY_1;

   assign opcode_WAY_0 = opcode_t'(instr_F_WAY_0[6:0]);
   assign opcode_WAY_1 = opcode_t'(instr_F_WAY_1[6:0]);

   logic check_way_0_cf, check_way_1_cf;

   assign check_way_0_cf = (opcode_WAY_0 == OP_JAL || opcode_WAY_0 == OP_JALR || opcode_WAY_0 == OP_BRANCH);
   assign check_way_1_cf = (opcode_WAY_1 == OP_JAL || opcode_WAY_1 == OP_JALR || opcode_WAY_1 == OP_BRANCH);

   always_comb begin
      if (~check_way_0_cf & ~check_way_1_cf) 
        begin
          predicted_WAY_0_F = 1;
          predicted_bcond_F = 0;
        end
      else if (~check_way_0_cf & check_way_1_cf & predicted_bcond_F_WAY_1) 
        begin
          predicted_WAY_0_F = 0;
          predicted_bcond_F = 1;
        end
      else if (~check_way_0_cf & check_way_1_cf & ~predicted_bcond_F_WAY_1)
        begin
          predicted_WAY_0_F = 0;
          predicted_bcond_F = 0;
        end
      else if (check_way_0_cf & ~check_way_1_cf & predicted_bcond_F_WAY_0)
        begin
          predicted_WAY_0_F = 1;
          predicted_bcond_F = 1;
        end
      else if (check_way_0_cf & ~check_way_1_cf & ~predicted_bcond_F_WAY_0)
        begin
          predicted_WAY_0_F = 1;
          predicted_bcond_F = 0;
        end
      else if (check_way_0_cf & check_way_1_cf)
        begin
         if (~predicted_bcond_F_WAY_0 & ~predicted_bcond_F_WAY_1) 
          begin
            predicted_WAY_0_F = 1;
            predicted_bcond_F = 0;
          end
         else if (~predicted_bcond_F_WAY_0 & predicted_bcond_F_WAY_1) 
          begin
            predicted_WAY_0_F = 0;
            predicted_bcond_F = 1;
          end
         else begin
            predicted_WAY_0_F = 1;
            predicted_bcond_F = 1;
         end
       end
      
      else begin
          predicted_WAY_0_F = 1;
          predicted_bcond_F = 1;
      end
    end

endmodule: check_F_control_flow

//sel == 0, checking rs1
//sel == 1, checking rs2
//module that returns true if instruction uses rs1 or rs2 and rs1/rs2 != X0
module waitForRs(
  input logic[31:0] instr,
  input logic sel,
  output logic using);

  opcode_t opcode;
  assign opcode = opcode_t'(instr[6:0]);

  logic [4:0] rs1, rs2;
  assign rs1 = instr[19:15];
  assign rs2 = instr[24:20];

  always_comb begin
    if(~sel) begin
      unique case (opcode)
        OP_OP:     using = (rs1 != X0);
        OP_IMM:    using = (rs1 != X0);
        OP_LOAD:   using = (rs1 != X0);
        OP_STORE:  using = (rs1 != X0);
        OP_LUI:    using = 0;
        OP_AUIPC:  using = 0;
        OP_JAL:    using = 0;
        OP_JALR:   using = (rs1 != X0);
        OP_BRANCH: using = (rs1 != X0);
        OP_SYSTEM: using = 1;
        default: using = 0;
      endcase
    end
    else begin
      unique case (opcode)
        OP_OP:     using = (rs2 != X0);
        OP_IMM:    using = 0;
        OP_LOAD:   using = 0;
        OP_STORE:  using = (rs2 != X0);
        OP_LUI:    using = 0;
        OP_AUIPC:  using = 0;
        OP_JAL:    using = 0;
        OP_JALR:   using = 0;
        OP_BRANCH: using = (rs2 != X0);
        OP_SYSTEM: using = 0;
        default: using = 0;
      endcase
    end
  end
endmodule: waitForRs


module bcond_mod(
  input logic [31:0] alu_out_E,
  input ctrl_signals_t ctrl_signals_E,
  output logic bcond);

  always_comb begin
      if(ctrl_signals_E.imm_mode==IMM_SB) begin
        unique case (ctrl_signals_E.btype)
          FUNCT3_BEQ: bcond = (alu_out_E==0);
          FUNCT3_BNE: bcond = !(alu_out_E==0);
          FUNCT3_BLT: bcond = (alu_out_E==1);
          FUNCT3_BGE: bcond = !(alu_out_E==1);
          FUNCT3_BLTU: bcond = (alu_out_E == 1);
          FUNCT3_BGEU: bcond = !(alu_out_E == 1);
          default: bcond = 0;
        endcase
      end
      else bcond = 0;
    end

endmodule: bcond_mod

