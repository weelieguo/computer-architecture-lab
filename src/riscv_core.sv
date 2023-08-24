/**
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the core part of the processor, and is responsible for executing the
 * instructions and updating the CPU state appropriately.
 *
 * This is where you can start to add code and make modifications to fully
 * implement the processor. You can add any additional files or change and
 * delete files as you need to implement the processor, provided that they are
 * under the src directory. You may not change any files outside the src
 * directory. The only requirement is that there is a riscv_core module with the
 * interface defined below, with the same port names as below.
 *
 * The Makefile will automatically find any files you add, provided they are
 * under the src directory and have either a *.v, *.vh, or *.sv extension. The
 * files may be nested in subdirectories under the src directory as well.
 * Additionally, the build system sets up the include paths so that you can
 * place header files (*.vh) in any subdirectory in the src directory, and
 * include them from anywhere else inside the src directory.
 *
 * The compiler and synthesis tools support both Verilog and System Verilog
 * constructs and syntax, so you can write either Verilog or System Verilog
 * code, or mix both as you please.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_abi.vh"             // ABI registers and definitions
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "memory_segments.vh"       // Memory segment starting addresses

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

/* A quick switch to enable/disable tracing. Comment out to disable. Please
 * comment this out before submitting your code. You'll also want to comment
 * this out for longer tests, as it will make them run much faster. */
`define TRACE

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/**
 * The core of the RISC-V processor, everything except main memory.
 *
 * This is the RISC-V processor, which, each cycle, fetches the next
 * instruction, executes it, and then updates the register file, memory,
 * and register file appropriately.
 *
 * The memory that the processor interacts with is dual-ported with a
 * single-cycle synchronous write and combinational read. One port is used to
 * fetch instructions, while the other is for loading and storing data.
 *
 * Inputs:
 *  - clk               The global clock for the processor.
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr_mem_excpt   Indicates that an invalid instruction address was given
 *                      to memory.
 *  - data_mem_excpt    Indicates that an invalid address was given to the data
 *                      memory during a load and/or store operation.
 *  - instr             The instruction loaded loaded from the instr_addr
 *                      address in memory.
 *  - data_load         The data loaded from the data_addr address in memory.
 *
 * Outputs:
 *  - data_load_en      Indicates that data from the data_addr address in
 *                      memory should be loaded.
 *  - halted            Indicates that the processor has stopped because of a
 *                      syscall or exception. Used to indicate to the testbench
 *                      to end simulation. Must be held until next clock cycle.
 *  - data_store_mask   Byte-enable bit mask  signal indicating which bytes of data_store
 *                      should be written to the data_addr address in memory.
 *  - instr_addr        The address of the instruction to load from memory.
 *  - instr_stall       stall instruction load from memory if multicycle.
 *  - data_addr         The address of the data to load or store from memory.
 *  - data_stall        stall data load from memory if multicycle.
 *  - data_store        The data to store to the data_addr address in memory.
 **/
module riscv_core
    (input  logic           clk, rst_l, instr_mem_excpt, data_mem_excpt,
     input  logic [2:0] [31:0] instr, 
     input  logic [2:0] [31:0] data_load,
     output logic           data_load_en, halted,
     output logic [3:0]     data_store_mask,
     output logic [29:0]    instr_addr, data_addr,
     output logic           instr_stall, data_stall,
     output logic [31:0]    data_store);

    //for all data load, store and stall signals, why is there only one bit rather than multiple for superscalar?
    //load data one by one, data_load is going to return consecutive memory data in mem starting from data_addr
    //if two consecutive lw instructions or sw instructions, need to stall WAY_1 until WAY_0 finishes
    //for stalling, need to add logic to make WAY_0 ignore stall but not WAY_1, possibly two stalls stall_WAY_0 and stall_WAY_1
    //output instr_stall & data_stall if either WAY_0 or WAY_1 needs to stall
    
    /* Import the ISA field types, and the argument to ecall to halt the
     * simulator, and the start of the user text segment. */
    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;
    import RISCV_UArch::SUPERSCALAR_WAYS;


    logic stall_WAY_0, stall_WAY_1, stall_F_D_WAY_0, halt_next, should_halt;
    //values needed for IF stage: instr_addr_F, pc_F, (output) instr_D 
    //values needed for ID stage: pc_D, npc+4_D, rf1, rs1_D, rs2_D, (output) rs1_data_E, (output) rs2_data_E, riscv_decode, (output) cntrl signals_E
    //values needed for EX stage: pc_E, immediate values_E, npc+4_E, (output) alu_out_M, (output) mult_out_M, (output) rs2_data_M,
    //values needed for MEM stage: npc+4_M, (output) alu_out_W, (output) data_load_W
    //values needed for WB stage: 

    
    logic [31:0] instr_F_WAY_0, instr_D_WAY_0, instr_E_WAY_0, instr_M_WAY_0, instr_W_WAY_0;
    logic [31:0] instr_F_WAY_1, instr_D_WAY_1, instr_E_WAY_1, instr_M_WAY_1, instr_W_WAY_1;
    logic mem_concur, mem_concur_D, rf_write_way_0, rf_write_way_0_D;

    assign instr_F_WAY_0 = instr[0];
    assign instr_F_WAY_1 = instr[1];

    // Manage the value of the PC, don't increment if the processor is halted
    logic [31:0]    pc_F;
    logic [31:0]    pc_F_WAY_0, pc_D_WAY_0, pc_E_WAY_0, pc_M_WAY_0, pc_W_WAY_0;
    logic [31:0]    npc_plus4_F_WAY_0, npc_plus4_D_WAY_0;  
    logic [31:0]    npc_offset_E_WAY_0, npc_offset_M_WAY_0, npc_offset_W_WAY_0;
    logic [31:0]    next_pc_WAY_0;
    logic [31:0]    boffset_E_WAY_0, boffset_M_WAY_0, boffset_W_WAY_0;
    logic 	    bcond_WAY_0, flush, flush_M;

    logic [31:0]    pc_F_WAY_1, pc_D_WAY_1, pc_E_WAY_1, pc_M_WAY_1, pc_W_WAY_1;
    logic [31:0]    npc_plus4_F_WAY_1, npc_plus4_D_WAY_1;  
    logic [31:0]    npc_offset_E_WAY_1, npc_offset_M_WAY_1, npc_offset_W_WAY_1;
    logic [31:0]    next_pc_WAY_1;
    logic [31:0]    boffset_E_WAY_1, boffset_M_WAY_1, boffset_W_WAY_1;
    logic 	    bcond_WAY_1;

    mem_concurrency MEM_Con_b (.*);
    check_rfWrite_mod rfWrite_b (.*);

    register #(1'b1) Mem_Concur_Register_D(.clk, .rst_l, .en(~halted), .clear(flush), .D(mem_concur),
            .Q(mem_concur_D));

    register #(1'b1) rf_Write_Register_D(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(rf_write_way_0),
            .Q(rf_write_way_0_D));

    register #(1'b1) halt_REG (.clk, .rst_l, .en(~halted), .clear(flush), .D(halt_next),
            .Q(should_halt));

    //instr stage registers WAY 0
    register #($bits(pc_F)) Instr_Register_D_WAY_0(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(instr_F_WAY_0),
            .Q(instr_D_WAY_0));

    register #($bits(pc_F)) Instr_Register_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(instr_D_WAY_0),
            .Q(instr_E_WAY_0));

    register #($bits(pc_F)) Instr_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(instr_E_WAY_0),
            .Q(instr_M_WAY_0));

    register #($bits(pc_F)) Instr_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(instr_M_WAY_0),
            .Q(instr_W_WAY_0));

    
    //instr stage registers WAY 1
    register #($bits(pc_F)) Instr_Register_D_WAY_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(instr_F_WAY_1),
            .Q(instr_D_WAY_1));

    register #($bits(pc_F)) Instr_Register_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(instr_D_WAY_1),
            .Q(instr_E_WAY_1));

    register #($bits(pc_F)) Instr_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(instr_E_WAY_1),
            .Q(instr_M_WAY_1));

    register #($bits(pc_F)) Instr_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(instr_M_WAY_1),
            .Q(instr_W_WAY_1));
    
    //boffset stage values
    assign boffset_E_WAY_0={{20{instr_E_WAY_0[31]}},instr_E_WAY_0[7],instr_E_WAY_0[30:25],instr_E_WAY_0[11:8],1'b0};
    assign boffset_M_WAY_0={{20{instr_M_WAY_0[31]}},instr_M_WAY_0[7],instr_M_WAY_0[30:25],instr_M_WAY_0[11:8],1'b0};
    assign boffset_W_WAY_0={{20{instr_W_WAY_0[31]}},instr_W_WAY_0[7],instr_W_WAY_0[30:25],instr_W_WAY_0[11:8],1'b0};

    assign boffset_E_WAY_1={{20{instr_E_WAY_1[31]}},instr_E_WAY_1[7],instr_E_WAY_1[30:25],instr_E_WAY_1[11:8],1'b0};
    assign boffset_M_WAY_1={{20{instr_M_WAY_1[31]}},instr_M_WAY_1[7],instr_M_WAY_1[30:25],instr_M_WAY_1[11:8],1'b0};
    assign boffset_W_WAY_1={{20{instr_W_WAY_1[31]}},instr_W_WAY_1[7],instr_W_WAY_1[30:25],instr_W_WAY_1[11:8],1'b0};

    logic [31:0]    alu_out_E_WAY_0, alu_out_M_WAY_0, alu_out_W_WAY_0;
    logic [31:0]    alu_out_E_WAY_1, alu_out_M_WAY_1, alu_out_W_WAY_1;

    //alu_out stage registers
    register #($bits(pc_F)) ALUO_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(alu_out_E_WAY_0),
            .Q(alu_out_M_WAY_0));

    register #($bits(pc_F)) ALUO_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(alu_out_M_WAY_0),
            .Q(alu_out_W_WAY_0));

    register #($bits(pc_F)) ALUO_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(alu_out_E_WAY_1),
            .Q(alu_out_M_WAY_1));

    register #($bits(pc_F)) ALUO_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(alu_out_M_WAY_1),
            .Q(alu_out_W_WAY_1));

    // Branch Prediction Logic
    logic [31:0] tagPC_F_WAY_0, tagPC_D, BTB_nextPC_WAY_0, BTB_nextPC_WAY_1;
    logic [31:0] tagPC_F_WAY_1;
    logic [1:0] history_WAY_0, history_D_WAY_0, history_E_WAY_0, updated_history_WAY_0;
    logic [1:0] history_WAY_1, history_D_WAY_1, history_E_WAY_1, updated_history_WAY_1;
    logic BTB_write_WAY_0, BTB_write_WAY_1, predicted_bcond_F_WAY_0, hysteresis_en_WAY_0, taken_WAY_0;
    logic predicted_bcond_F_WAY_1, hysteresis_en_WAY_1, taken_WAY_1;
    logic predicted_bcond_E, predicted_bcond_D, predicted_bcond_F;
    logic predicted_WAY_0_E, predicted_WAY_0_F, predicted_WAY_0_D;

    assign hysteresis_en_WAY_0 = (ctrl_signals_E_WAY_0.pc_source == PC_cond) || (ctrl_signals_E_WAY_0.pc_source == PC_uncond);
    assign hysteresis_en_WAY_1 = (ctrl_signals_E_WAY_1.pc_source == PC_cond) || (ctrl_signals_E_WAY_1.pc_source == PC_uncond);
    
    assign taken_WAY_0 = ((ctrl_signals_E_WAY_0.pc_source == PC_uncond) || bcond_WAY_0);
    assign taken_WAY_1 = ((ctrl_signals_E_WAY_1.pc_source == PC_uncond) || bcond_WAY_1);

    check_F_control_flow  predict_control_block (.*);
    
    update UP_DATE_WAY_0 (.hysteresis_en(hysteresis_en_WAY_0), .history_E(history_E_WAY_0),.taken(taken_WAY_0), .updated_history(updated_history_WAY_0));
    update UP_DATE_WAY_1 (.hysteresis_en(hysteresis_en_WAY_1), .history_E(history_E_WAY_1),.taken(taken_WAY_1), .updated_history(updated_history_WAY_1));

    predicted_bcond pbcond_WAY_0 (.history(history_WAY_0), .predicted_bcond_F(predicted_bcond_F_WAY_0));
    predicted_bcond pbcond_WAY_1 (.history(history_WAY_1), .predicted_bcond_F(predicted_bcond_F_WAY_1));
    
    logic [1:0] [61:0] BTB_write_data; 
    logic [1:0] [61:0] BTB_read_data;

    assign tagPC_F_WAY_0 = {BTB_read_data[0][61:32], 2'b0};
    assign history_WAY_0 = BTB_read_data[0][31:30];
    assign BTB_nextPC_WAY_0 = {BTB_read_data[0][29:0], 2'b0};

    assign tagPC_F_WAY_1 = {BTB_read_data[1][61:32], 2'b0};
    assign history_WAY_1 = BTB_read_data[1][31:30];
    assign BTB_nextPC_WAY_1 = {BTB_read_data[1][29:0], 2'b0};


    logic [1:0][7:0] BTB_write_addr;
    assign BTB_write_addr[0] = pc_E_WAY_0[9:2];
    assign BTB_write_data[0] = {pc_E_WAY_0[31:2], updated_history_WAY_0, next_pc_WAY_0[31:2]};
      
    assign BTB_write_addr[1] = pc_E_WAY_1[9:2];
    assign BTB_write_data[1] = {pc_E_WAY_1[31:2], updated_history_WAY_1, next_pc_WAY_0[31:2]};

    logic [1:0] [7:0] BTB_read_addr;
   
    assign BTB_read_addr[0] = pc_F_WAY_0[9:2];
    assign BTB_read_addr[1] = pc_F_WAY_1[9:2];

    //two asynchronous reads,  synchronous write
    riscv_sram BTB (.clk, .rst_l, .we_WAY_0(BTB_write_WAY_0), .we_WAY_1(BTB_write_WAY_1), .read_addr(BTB_read_addr), .write_addr(BTB_write_addr), 
              .read_data(BTB_read_data), .write_data(BTB_write_data));

    
    

    
    
    // Next PC logic
    register #($bits(1'b1)) predict_bond_Register_D(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(predicted_bcond_F),
            .Q(predicted_bcond_D));

    register #($bits(1'b1)) predict_bond_Register_E(.clk, .rst_l, .en(~halted), .clear(flush), .D(predicted_bcond_D),
            .Q(predicted_bcond_E));

    register #($bits(history_WAY_0)) history_REG_D_WAY_0(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(history_WAY_0),
            .Q(history_D_WAY_0));

    register #($bits(history_WAY_0)) history_REG_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(history_D_WAY_0),
            .Q(history_E_WAY_0));

    register #($bits(history_WAY_0)) history_REG_D_WAY_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(history_WAY_1),
            .Q(history_D_WAY_1));

    register #($bits(history_WAY_0)) history_REG_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(history_D_WAY_1),
            .Q(history_E_WAY_1));

    register #(1'b1) predicted_WAY_0_REG_D (.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(predicted_WAY_0_F),
            .Q(predicted_WAY_0_D));
 
    register #(1'b1) predicted_WAY_0_REG_E (.clk, .rst_l, .en(~halted), .clear(flush), .D(predicted_WAY_0_D),
            .Q(predicted_WAY_0_E));

    nextPCmod nextPC_Mod (.*);

    logic [31:0] predicted_nextPC_F_WAY_0, predicted_nextPC_D_WAY_0, predicted_nextPC_E;

    register #($bits(predicted_nextPC_F_WAY_0)) predicted_nextPC_D_REG(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), 
                                                                       .clear(flush), .D(predicted_nextPC_F_WAY_0),
            .Q(predicted_nextPC_D_WAY_0));

    register #($bits(predicted_nextPC_F_WAY_0)) predicted_nextPC_E_REG(.clk, .rst_l, .en(~halted), .clear(flush), .D(predicted_nextPC_D_WAY_0),
            .Q(predicted_nextPC_E));


    predicted_nextPC predictedNPC (.*);
    
    logic [31:0] next_pc_E_WAY_0;
    logic choose_E;
    branchP BRANCH_B (.*);


   
  

    // npc_offset adder
    adder #($bits(pc_F)) Offset_PC_Adder_WAY_0(.A(pc_E_WAY_0), .B(boffset_E_WAY_0), .cin(1'b0),
            .sum(npc_offset_E_WAY_0), .cout());

    adder #($bits(pc_F)) Offset_PC_Adder_WAY_1(.A(pc_E_WAY_1), .B(boffset_E_WAY_1), .cin(1'b0),
            .sum(npc_offset_E_WAY_1), .cout());

    //npc_offset stage registers

    register #($bits(pc_F)) NPCO_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(npc_offset_E_WAY_0),
            .Q(npc_offset_M_WAY_0));

    register #($bits(pc_F)) NPCO_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(npc_offset_M_WAY_0),
            .Q(npc_offset_W_WAY_0));

    register #($bits(pc_F)) NPCO_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(npc_offset_E_WAY_1),
            .Q(npc_offset_M_WAY_1));

    register #($bits(pc_F)) NPCO_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(npc_offset_M_WAY_1),
            .Q(npc_offset_W_WAY_1));    

    //pc stage registers WAY 0
    register #($bits(pc_F), USER_TEXT_START) PC_Register_F_WAY_0(.clk, .rst_l, .en(~halted & (~stall_F_D_WAY_0 || flush)), .clear(1'b0), .D(next_pc_WAY_0),
            .Q(pc_F_WAY_0));

    register #($bits(pc_F)) PC_Register_D_WAY_0(.clk, .rst_l, .en(~halted & ~stall_F_D_WAY_0), .clear(flush), .D(pc_F_WAY_0),
            .Q(pc_D_WAY_0));

    register #($bits(pc_F)) PC_Register_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(pc_D_WAY_0),
            .Q(pc_E_WAY_0));

    register #($bits(pc_F)) PC_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(pc_E_WAY_0),
            .Q(pc_M_WAY_0));

    register #($bits(pc_F)) PC_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(pc_M_WAY_0),
            .Q(pc_W_WAY_0));

    assign pc_F_WAY_1 = pc_F_WAY_0 + 4;

    //pc stage registers WAY 1
    register #($bits(pc_F)) PC_Register_D_WAY_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(pc_F_WAY_1),
            .Q(pc_D_WAY_1));

    register #($bits(pc_F)) PC_Register_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(pc_D_WAY_1),
            .Q(pc_E_WAY_1));

    register #($bits(pc_F)) PC_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(pc_E_WAY_1),
            .Q(pc_M_WAY_1));

    register #($bits(pc_F)) PC_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(pc_M_WAY_1),
            .Q(pc_W_WAY_1));

    ///////


    logic [31:0] data_load_M;
    logic [31:0] data_load_W;

    assign data_load_M = data_load[0];
    

    //data_load stage registers
    register #($bits(data_load_W)) Data_load_Register_W(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(data_load_M),
            .Q(data_load_W));


    assign instr_addr       = pc_F_WAY_0[31:2];

   


    ctrl_signals_t  received_ctrl_signals_D_WAY_0, received_ctrl_signals_D_WAY_1; 

    // Decode the instruction and generate the control signals
    ctrl_signals_t  ctrl_signals_D_WAY_0, ctrl_signals_D_WAY_1;
    ctrl_signals_t  ctrl_signals_E_WAY_0, ctrl_signals_E_WAY_1;
    ctrl_signals_t  ctrl_signals_M_WAY_0, ctrl_signals_M_WAY_1;
    ctrl_signals_t  ctrl_signals_W_WAY_0, ctrl_signals_W_WAY_1;

    riscv_decode Decoder_WAY_0(.rst_l(rst_l & (instr_D_WAY_0 != 0)), .instr(instr_D_WAY_0),
            .ctrl_signals(received_ctrl_signals_D_WAY_0));

    riscv_decode Decoder_WAY_1(.rst_l(rst_l & (instr_D_WAY_1 != 0)), .instr(instr_D_WAY_1),
            .ctrl_signals(received_ctrl_signals_D_WAY_1));

    logic no_op_WAY_0_E, no_op_WAY_0, concur, prev_concur;

    //Allows us to NO_OP stalled ctrl_signals in pipeline
    ctrlSignals ctrlSignals_mod_WAY_0 (.received_ctrl_signals_D(received_ctrl_signals_D_WAY_0), 
                                       .ctrl_signals_D(ctrl_signals_D_WAY_0), .stall(stall_WAY_0 || no_op_WAY_0_E));

    ctrlSignals ctrlSignals_mod_WAY_1 (.received_ctrl_signals_D(received_ctrl_signals_D_WAY_1), 
                                       .ctrl_signals_D(ctrl_signals_D_WAY_1), .stall(stall_WAY_1));

    //ctrl_signals stage registers
    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(ctrl_signals_D_WAY_0),
            .Q(ctrl_signals_E_WAY_0));

    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(ctrl_signals_E_WAY_0),
            .Q(ctrl_signals_M_WAY_0));

    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(ctrl_signals_M_WAY_0),
            .Q(ctrl_signals_W_WAY_0));

    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(ctrl_signals_D_WAY_1),
            .Q(ctrl_signals_E_WAY_1));

    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(ctrl_signals_E_WAY_1),
            .Q(ctrl_signals_M_WAY_1));

    register #($bits(ctrl_signals_D_WAY_0)) Ctrl_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(ctrl_signals_M_WAY_1),
            .Q(ctrl_signals_W_WAY_1));

    register #(1'b1) No_OP_Register_E(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(no_op_WAY_0),
            .Q(no_op_WAY_0_E));

    register #(1'b1) Concur_Register_E(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(concur),
            .Q(prev_concur));

    //register file stage signals and assignment
    logic [4:0]   rs1_D_WAY_0, rs1_E_WAY_0;
    logic [4:0]   rs2_D_WAY_0, rs2_E_WAY_0;
    logic [4:0]   rd_D_WAY_0, rd_E_WAY_0, rd_M_WAY_0, rd_W_WAY_0;
    logic [31:0]  rs1_data_D_WAY_0, rs1_data_E_WAY_0;
    logic [31:0]  rs2_data_D_WAY_0, rs2_data_E_WAY_0, rs2_data_M_WAY_0; 
    logic [31:0]  rd_data_W_WAY_0;

    logic [4:0]   rs1_D_WAY_1, rs1_E_WAY_1;
    logic [4:0]   rs2_D_WAY_1, rs2_E_WAY_1;
    logic [4:0]   rd_D_WAY_1, rd_E_WAY_1, rd_M_WAY_1, rd_W_WAY_1;
    logic [31:0]  rs1_data_D_WAY_1, rs1_data_E_WAY_1;
    logic [31:0]  rs2_data_D_WAY_1, rs2_data_E_WAY_1, rs2_data_M_WAY_1; 
    logic [31:0]  rd_data_W_WAY_1;

    //Forwarding data signals
    logic [31:0] rs1_data_final_E_WAY_0, rs2_data_final_E_WAY_0, rs1_data_final_M_WAY_0, rs1_data_final_W_WAY_0;
    logic [31:0] rs1_data_final_E_WAY_1, rs2_data_final_E_WAY_1, rs1_data_final_M_WAY_1, rs1_data_final_W_WAY_1;

    //rf data stage registers WAY 0
    register #($bits(rd_data_W_WAY_0)) Rs1_Data_Register_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(rs1_data_D_WAY_0),
            .Q(rs1_data_E_WAY_0));

    register #($bits(rd_data_W_WAY_0)) Rs2_Data_Register_E_WAY_0(.clk, .rst_l, .en(~halted), .clear(flush), .D(rs2_data_D_WAY_0),
            .Q(rs2_data_E_WAY_0));

    register #($bits(rd_data_W_WAY_0)) Rs2_Data_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rs2_data_final_E_WAY_0),
            .Q(rs2_data_M_WAY_0));

    register #($bits(rd_data_W_WAY_0)) Rs1_Data_Final_Register_M_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rs1_data_final_E_WAY_0),
            .Q(rs1_data_final_M_WAY_0));

    register #($bits(rd_data_W_WAY_0)) Rs1_Data_Final_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rs1_data_final_M_WAY_0),
            .Q(rs1_data_final_W_WAY_0));

    //rf data stage registers WAY 1
    register #($bits(rd_data_W_WAY_1)) Rs1_Data_Register_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(rs1_data_D_WAY_1),
            .Q(rs1_data_E_WAY_1));

    register #($bits(rd_data_W_WAY_1)) Rs2_Data_Register_E_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush), .D(rs2_data_D_WAY_1),
            .Q(rs2_data_E_WAY_1));

    register #($bits(rd_data_W_WAY_1)) Rs2_Data_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(rs2_data_final_E_WAY_1),
            .Q(rs2_data_M_WAY_1));

    register #($bits(rd_data_W_WAY_1)) Rs1_Data_Final_Register_M_WAY_1(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(rs1_data_final_E_WAY_1),
            .Q(rs1_data_final_M_WAY_1));

    register #($bits(rd_data_W_WAY_1)) Rs1_Data_Final_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rs1_data_final_M_WAY_1),
            .Q(rs1_data_final_W_WAY_1));

    logic forward_rs1_E_WAY_0_0, forward_rs2_E_WAY_0_0,forward_rs1_M_WAY_0_0, forward_rs2_M_WAY_0_0;
    logic forward_rs1_E_WAY_0_1, forward_rs2_E_WAY_0_1,forward_rs1_M_WAY_0_1, forward_rs2_M_WAY_0_1;
    logic forward_rs1_E_WAY_1_0, forward_rs2_E_WAY_1_0,forward_rs1_M_WAY_1_0, forward_rs2_M_WAY_1_0;
    logic forward_rs1_E_WAY_1_1, forward_rs2_E_WAY_1_1,forward_rs1_M_WAY_1_1, forward_rs2_M_WAY_1_1;

    logic forward_rs1_E_E_WAY_0_0, forward_rs2_E_E_WAY_0_0,forward_rs1_M_E_WAY_0_0, forward_rs2_M_E_WAY_0_0;
    logic forward_rs1_E_E_WAY_0_1, forward_rs2_E_E_WAY_0_1,forward_rs1_M_E_WAY_0_1, forward_rs2_M_E_WAY_0_1;
    logic forward_rs1_E_E_WAY_1_0, forward_rs2_E_E_WAY_1_0,forward_rs1_M_E_WAY_1_0, forward_rs2_M_E_WAY_1_0;
    logic forward_rs1_E_E_WAY_1_1, forward_rs2_E_E_WAY_1_1,forward_rs1_M_E_WAY_1_1, forward_rs2_M_E_WAY_1_1;

    logic [31:0] mult_out_M_WAY_0, mult_out_W_WAY_0, mult_out_M_WAY_1, mult_out_W_WAY_1;    

    //Forward registers WAY 0 0
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_E_WAY_0_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs1_E_WAY_0_0),
            .Q(forward_rs1_E_E_WAY_0_0));
    
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_M_WAY_0_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs1_M_WAY_0_0),
            .Q(forward_rs1_M_E_WAY_0_0));

    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_E_WAY_0_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs2_E_WAY_0_0),
            .Q(forward_rs2_E_E_WAY_0_0));
    
    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_M_WAY_0_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs2_M_WAY_0_0),
            .Q(forward_rs2_M_E_WAY_0_0));

    //Forward registers WAY 0 1
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_E_WAY_0_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs1_E_WAY_0_1),
            .Q(forward_rs1_E_E_WAY_0_1));
    
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_M_WAY_0_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs1_M_WAY_0_1),
            .Q(forward_rs1_M_E_WAY_0_1));

    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_E_WAY_0_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs2_E_WAY_0_1),
            .Q(forward_rs2_E_E_WAY_0_1));
    
    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_M_WAY_0_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs2_M_WAY_0_1),
            .Q(forward_rs2_M_E_WAY_0_1));

    //Forward registers WAY 1 0
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_E_WAY_1_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush | flush_M), .D(forward_rs1_E_WAY_1_0),
            .Q(forward_rs1_E_E_WAY_1_0));
    
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_M_WAY_1_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs1_M_WAY_1_0),
            .Q(forward_rs1_M_E_WAY_1_0));

    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_E_WAY_1_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush | flush_M), .D(forward_rs2_E_WAY_1_0),
            .Q(forward_rs2_E_E_WAY_1_0));
    
    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_M_WAY_1_0(.clk, .rst_l, .en(~halted & ~stall_WAY_0), .clear(flush), .D(forward_rs2_M_WAY_1_0),
            .Q(forward_rs2_M_E_WAY_1_0));

    //Forward registers WAY 1 1
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_E_WAY_1_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush | flush_M), .D(forward_rs1_E_WAY_1_1),
            .Q(forward_rs1_E_E_WAY_1_1));
    
    register #($bits(forward_rs1_E_WAY_0_0)) FW_REG1_M_WAY_1_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs1_M_WAY_1_1),
            .Q(forward_rs1_M_E_WAY_1_1));

    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_E_WAY_1_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush | flush_M), .D(forward_rs2_E_WAY_1_1),
            .Q(forward_rs2_E_E_WAY_1_1));
    
    register #($bits(forward_rs2_E_WAY_0_0)) FW_REG2_M_WAY_1_1(.clk, .rst_l, .en(~halted & ~stall_WAY_1), .clear(flush), .D(forward_rs2_M_WAY_1_1),
            .Q(forward_rs2_M_E_WAY_1_1));

    //start
  
    forward FORWARD_DATA (.*);



    chooseRS1 rs1_D_choose_WAY_0 (.rs1(rs1_D_WAY_0), .ctrl_signals(ctrl_signals_D_WAY_0), .instr(instr_D_WAY_0));
    chooseRS1 rs1_E_choose_WAY_0 (.rs1(rs1_E_WAY_0), .ctrl_signals(ctrl_signals_E_WAY_0), .instr(instr_E_WAY_0));

    chooseRS1 rs1_D_choose_WAY_1 (.rs1(rs1_D_WAY_1), .ctrl_signals(ctrl_signals_D_WAY_1), .instr(instr_D_WAY_1));
    chooseRS1 rs1_E_choose_WAY_1 (.rs1(rs1_E_WAY_1), .ctrl_signals(ctrl_signals_E_WAY_1), .instr(instr_E_WAY_1));
    
    assign  rs2_D_WAY_0 = instr_D_WAY_0[24:20];
    assign  rs2_E_WAY_0 = instr_E_WAY_0[24:20];

    assign  rs2_D_WAY_1 = instr_D_WAY_1[24:20];
    assign  rs2_E_WAY_1 = instr_E_WAY_1[24:20];

    assign  rd_D_WAY_0 = instr_D_WAY_0[11:7];
    assign  rd_E_WAY_0 = instr_E_WAY_0[11:7];
    assign  rd_M_WAY_0 = instr_M_WAY_0[11:7];
    assign  rd_W_WAY_0 = instr_W_WAY_0[11:7];

    assign  rd_D_WAY_1 = instr_D_WAY_1[11:7];
    assign  rd_E_WAY_1 = instr_E_WAY_1[11:7];
    assign  rd_M_WAY_1 = instr_M_WAY_1[11:7];
    assign  rd_W_WAY_1 = instr_W_WAY_1[11:7];
   
   
   logic [2:0][31:0] rd_data_W_con;
   assign rd_data_W_con[0] = rd_data_W_WAY_0;
   assign rd_data_W_con[1] = rd_data_W_WAY_1;
   assign rd_data_W_con[2] = 32'b0;

   logic [2:0] rd_we_con;
   assign rd_we_con[0] = ctrl_signals_W_WAY_0.rfWrite;
   assign rd_we_con[1] = ctrl_signals_W_WAY_1.rfWrite;
   assign rd_we_con[2] = 1'b0;

   logic [2:0][4:0] rs1_D_con, rs2_D_con, rd_W_con;
   logic [2:0][31:0] rs1_data_D, rs2_data_D;
   
   assign rs1_D_con[0] = rs1_D_WAY_0;
   assign rs1_D_con[1] = rs1_D_WAY_1;
   assign rs1_D_con[2] = 5'b0;

   assign rs2_D_con[0] = rs2_D_WAY_0;
   assign rs2_D_con[1] = rs2_D_WAY_1;
   assign rs2_D_con[2] = 5'b0;
   
   assign rd_W_con[0] = rd_W_WAY_0;
   assign rd_W_con[1] = rd_W_WAY_1;
   assign rd_W_con[2] = 5'b0;

   assign rs1_data_D_WAY_0 = rs1_data_D[0];
   assign rs1_data_D_WAY_1 = rs1_data_D[1];

   assign rs2_data_D_WAY_0 = rs2_data_D[0];
   assign rs2_data_D_WAY_1 = rs2_data_D[1];
   

   //takes in D stage inputs, other than rd signals (W stage), outputs E stage outputs
   register_file #(.FORWARD(1))rf1 (.rd_we(rd_we_con), .rs1(rs1_D_con), .rs2(rs2_D_con), .rd(rd_W_con),
                      .rs1_data(rs1_data_D), .rs2_data(rs2_data_D), .rd_data(rd_data_W_con), .*);

    // Execute the instruction, performing the needed ALU operation
    
    logic [31:0]    se_immediate_E_WAY_0, alu_src2_WAY_0;
    logic [31:0]    se_immediate_E_WAY_1, alu_src2_WAY_1;
                                                            
    // set first parameter to 0 for combinatonal; to 1 for pipelined.  
                                  
    multcsa #(1, 32) multiplier_WAY_0 (.A(rs1_data_final_E_WAY_0), .B(rs2_data_final_E_WAY_0), .O(mult_out_M_WAY_0), .CLK(clk));
    multcsa #(1, 32) multiplier_WAY_1 (.A(rs1_data_final_E_WAY_1), .B(rs2_data_final_E_WAY_1), .O(mult_out_M_WAY_1), .CLK(clk));

    //mult_out stage registers
    register #($bits(pc_F)) MultO_Register_W_WAY_0(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(mult_out_M_WAY_0),
            .Q(mult_out_W_WAY_0));

    register #($bits(pc_F)) MultO_Register_W_WAY_1(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(mult_out_M_WAY_1),
            .Q(mult_out_W_WAY_1));

    //mux to choose between rs2 and immediate value for ALU
    //NEXT_ONE
    mux #(2, $bits(rs2_data_E_WAY_0)) ALU_Src2_Mux_WAY_0(.in({rs2_data_final_E_WAY_0, se_immediate_E_WAY_0}), .sel(!ctrl_signals_E_WAY_0.useImm),
            .out(alu_src2_WAY_0));
    riscv_alu ALU_WAY_0(.alu_src1(rs1_data_final_E_WAY_0), .alu_src2(alu_src2_WAY_0), .alu_op(ctrl_signals_E_WAY_0.alu_op),
            .alu_out(alu_out_E_WAY_0),.pc(pc_E_WAY_0));

    mux #(2, $bits(rs2_data_E_WAY_1)) ALU_Src2_Mux_WAY_1(.in({rs2_data_final_E_WAY_1, se_immediate_E_WAY_1}), .sel(!ctrl_signals_E_WAY_1.useImm),
            .out(alu_src2_WAY_1));
    riscv_alu ALU_WAY_1(.alu_src1(rs1_data_final_E_WAY_1), .alu_src2(alu_src2_WAY_1), .alu_op(ctrl_signals_E_WAY_1.alu_op),
            .alu_out(alu_out_E_WAY_1),.pc(pc_E_WAY_1));


    //bcond generation logic
    bcond_mod bmod_WAY_0 (.alu_out_E(alu_out_E_WAY_0), .ctrl_signals_E(ctrl_signals_E_WAY_0), .bcond(bcond_WAY_0));
    bcond_mod bmod_WAY_1 (.alu_out_E(alu_out_E_WAY_1), .ctrl_signals_E(ctrl_signals_E_WAY_1), .bcond(bcond_WAY_1));
 
   
    //immediate value masking
    logic [31:0] i_immediate_WAY_0, u_immediate_WAY_0, uj_l_immediate_WAY_0, uj_r_immediate_WAY_0, uj_immediate_WAY_0; 
    logic [31:0] s_immediate_WAY_0, sb_immediate_WAY_0, dx_immediate_WAY_0;

    logic [31:0] i_immediate_WAY_1, u_immediate_WAY_1, uj_l_immediate_WAY_1, uj_r_immediate_WAY_1, uj_immediate_WAY_1; 
    logic [31:0] s_immediate_WAY_1, sb_immediate_WAY_1, dx_immediate_WAY_1; 

    assign i_immediate_WAY_0 = {{21{instr_E_WAY_0[31]}}, instr_E_WAY_0[30:20]};
    assign u_immediate_WAY_0 = {{13{instr_E_WAY_0[31]}}, instr_E_WAY_0[30:12]};
    assign uj_l_immediate_WAY_0 = (instr_E_WAY_0[31] == 1) ? 32'b1111_1111_1111_0000_0000_0000_0000_0000:32'b0;
    assign uj_r_immediate_WAY_0 = {12'b0, instr_E_WAY_0[19:12], instr_E_WAY_0[20], instr_E_WAY_0[30:21], 1'b0};
    assign uj_immediate_WAY_0 = uj_l_immediate_WAY_0 | uj_r_immediate_WAY_0;
    assign s_immediate_WAY_0 = {{21{instr_E_WAY_0[31]}}, instr_E_WAY_0[30:25], instr_E_WAY_0[11:7]};
    assign sb_immediate_WAY_0 = {{20{instr_E_WAY_0[31]}}, instr_E_WAY_0[7], instr_E_WAY_0[30:25], instr_E_WAY_0[11:8], 1'b0};
    assign dx_immediate_WAY_0 = 32'bx;

    assign i_immediate_WAY_1 = {{21{instr_E_WAY_1[31]}}, instr_E_WAY_1[30:20]};
    assign u_immediate_WAY_1 = {{13{instr_E_WAY_1[31]}}, instr_E_WAY_1[30:12]};
    assign uj_l_immediate_WAY_1 = (instr_E_WAY_1[31] == 1) ? 32'b1111_1111_1111_0000_0000_0000_0000_0000:32'b0;
    assign uj_r_immediate_WAY_1 = {12'b0, instr_E_WAY_1[19:12], instr_E_WAY_1[20], instr_E_WAY_1[30:21], 1'b0};
    assign uj_immediate_WAY_1 = uj_l_immediate_WAY_1 | uj_r_immediate_WAY_1;
    assign s_immediate_WAY_1 = {{21{instr_E_WAY_1[31]}}, instr_E_WAY_1[30:25], instr_E_WAY_1[11:7]};
    assign sb_immediate_WAY_1 = {{20{instr_E_WAY_1[31]}}, instr_E_WAY_1[7], instr_E_WAY_1[30:25], instr_E_WAY_1[11:8], 1'b0};
    assign dx_immediate_WAY_1 = 32'bx;
    //mux to choose correct immediate value to use based on ctrl signals
    mux #(6, $bits(rs2_data_E_WAY_0)) ALU_Src3_Mux_WAY_0(.in({dx_immediate_WAY_0, uj_immediate_WAY_0, 
                                                u_immediate_WAY_0, sb_immediate_WAY_0, s_immediate_WAY_0, 
                                                i_immediate_WAY_0}), .sel(ctrl_signals_E_WAY_0.imm_mode),
                                           .out(se_immediate_E_WAY_0));

    mux #(6, $bits(rs2_data_E_WAY_1)) ALU_Src3_Mux_WAY_1(.in({dx_immediate_WAY_1, uj_immediate_WAY_1, 
                                                u_immediate_WAY_1, sb_immediate_WAY_1, s_immediate_WAY_1, 
                                                i_immediate_WAY_1}), .sel(ctrl_signals_E_WAY_1.imm_mode),
                                           .out(se_immediate_E_WAY_1)); 
    
     //forwarding logic
    
    forward_en rs1_forwardEn         (.rs_D_WAY_0(rs1_D_WAY_0), .rd_E_WAY_0(rd_E_WAY_0), .rd_M_WAY_0(rd_M_WAY_0),
                                      .rs_D_WAY_1(rs1_D_WAY_1), .rd_E_WAY_1(rd_E_WAY_1), .rd_M_WAY_1(rd_M_WAY_1), 
                                      .ctrl_signals_E_WAY_0(ctrl_signals_E_WAY_0), .ctrl_signals_M_WAY_0(ctrl_signals_M_WAY_0),
                                      .ctrl_signals_E_WAY_1(ctrl_signals_E_WAY_1), .ctrl_signals_M_WAY_1(ctrl_signals_M_WAY_1),
                                      .forward_rs_E_WAY_0_0(forward_rs1_E_WAY_0_0), .forward_rs_M_WAY_0_0(forward_rs1_M_WAY_0_0),
                                      .forward_rs_E_WAY_0_1(forward_rs1_E_WAY_0_1), .forward_rs_M_WAY_0_1(forward_rs1_M_WAY_0_1),
                                      .forward_rs_E_WAY_1_0(forward_rs1_E_WAY_1_0), .forward_rs_M_WAY_1_0(forward_rs1_M_WAY_1_0),
                                      .forward_rs_E_WAY_1_1(forward_rs1_E_WAY_1_1), .forward_rs_M_WAY_1_1(forward_rs1_M_WAY_1_1),.*);

    forward_en rs2_forwardEn         (.rs_D_WAY_0(rs2_D_WAY_0), .rd_E_WAY_0(rd_E_WAY_0), .rd_M_WAY_0(rd_M_WAY_0),
                                      .rs_D_WAY_1(rs2_D_WAY_1), .rd_E_WAY_1(rd_E_WAY_1), .rd_M_WAY_1(rd_M_WAY_1), 
                                      .ctrl_signals_E_WAY_0(ctrl_signals_E_WAY_0), .ctrl_signals_M_WAY_0(ctrl_signals_M_WAY_0),
                                      .ctrl_signals_E_WAY_1(ctrl_signals_E_WAY_1), .ctrl_signals_M_WAY_1(ctrl_signals_M_WAY_1),
                                      .forward_rs_E_WAY_0_0(forward_rs2_E_WAY_0_0), .forward_rs_M_WAY_0_0(forward_rs2_M_WAY_0_0),
                                      .forward_rs_E_WAY_0_1(forward_rs2_E_WAY_0_1), .forward_rs_M_WAY_0_1(forward_rs2_M_WAY_0_1),
                                      .forward_rs_E_WAY_1_0(forward_rs2_E_WAY_1_0), .forward_rs_M_WAY_1_0(forward_rs2_M_WAY_1_0),
                                      .forward_rs_E_WAY_1_1(forward_rs2_E_WAY_1_1), .forward_rs_M_WAY_1_1(forward_rs2_M_WAY_1_1),.*);
    
    //logic for handling stalls

    logic usingRS1_WAY_0, usingRS2_WAY_0;
    logic usingRS1_WAY_1, usingRS2_WAY_1;
    logic [31:0]    a0_value_WAY_0, a0_value_WAY_1;
  
    //incorrect, stall_WAY_0 and stall_WAY_1 now may be two separate things based on dependencies

    waitForRs waitRS1_WAY_0(.instr(instr_D_WAY_0), .sel(1'b0), .using(usingRS1_WAY_0));
    waitForRs waitRS2_WAY_0(.instr(instr_D_WAY_0), .sel(1'b1), .using(usingRS2_WAY_0));

    waitForRs waitRS1_WAY_1(.instr(instr_D_WAY_1), .sel(1'b0), .using(usingRS1_WAY_1));
    waitForRs waitRS2_WAY_1(.instr(instr_D_WAY_1), .sel(1'b1), .using(usingRS2_WAY_1));

    /* 
    stall_b STALL_Block (.rs1_D(rs1_D), .rd_E(rd_E), .rd_M(rd_M), 
                         .rd_W(rd_W), .rs2_D(rs2_D),
                         .ctrl_signals_D_syscall(ctrl_signals_D.syscall),
                         .ctrl_signals_E_memRead(ctrl_signals_E.memRead),
                         .ctrl_signals_E_syscall(ctrl_signals_E.syscall),
                         .ctrl_signals_M_syscall(ctrl_signals_M.syscall),
                         .ctrl_signals_W_syscall(ctrl_signals_W.syscall),
                         .ctrl_signals_E_alu_op(ctrl_signals_E.alu_op),
                         .ctrl_signals_E_rfWrite(ctrl_signals_E.rfWrite),
                         .ctrl_signals_M_rfWrite(ctrl_signals_M.rfWrite),
                         .ctrl_signals_W_rfWrite(ctrl_signals_W.rfWrite),
                         .usingRS1(usingRS1),
                         .usingRS2(usingRS2),
                         .data_stall(data_stall),
                         .instr_stall(instr_stall));     
    */

    logic stall_case_1, stall_case_2, prev_stall_case_1;

    register #($bits(prev_stall_case_1)) Prev_Stall_1_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(stall_case_1),
            .Q(prev_stall_case_1));

    /*
    register #($bits(prev_stall_case_1)) Prev_Stall_2_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(stall_case_2),
            .Q(prev_stall_case_2));

   */

    

    //will need to update this stall block in library
    old_stall_b STALL_Block (.*);


    //data storing logic and masking
    //incorrect, will need logic to choose between WAY_0 and WAY_1 for data_addr and data_load_en_W
    //same for store signals
    logic ldstr_E_WAY_0;
    logic ldstr_M_WAY_0;
    logic ldstr_W_WAY_0;
    logic data_load_en_M;
    logic [31:0] store_shift;
    logic [31:0] data_offset_M;
    logic [31:0] data_shifted_M;
    logic [31:0] data_res_M, data_res_W;
    logic [31:0] rd_data_M_WAY_0, rd_data_M_WAY_1;

    assign ldstr_E_WAY_0 = (ctrl_signals_E_WAY_0.memRead || ctrl_signals_E_WAY_0.memWrite);
    //assign ldstr_M_WAY_0 = (ctrl_signals_M_WAY_0.memRead || ctrl_signals_M_WAY_0.memWrite);
    assign data_load_en_M = data_load_en;

    //logic for choosing M stage load and store signals
    ldstr_M_signals ldstrM (.*);

    register #(1'b1) LDSTR_M_WAY_0_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(ldstr_E_WAY_0),
            .Q(ldstr_M_WAY_0));

    //register #(1'b1) LDSTR_W_WAY_0_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(ldstr_M_WAY_0),
    //        .Q(ldstr_W_WAY_0));

    //could reinstantiate ldstrM with W signals later
    //ldstr_W_signals ldstrW (.*);
    logic [31:0] data_store_E;
    logic [3:0] data_store_mask_E;

    dataStore dStore (.*);
  
    assign data_shifted_M = (data_load_M >> data_offset_M);

    register #($bits(data_res_M)) data_res_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(data_res_M),
            .Q(data_res_W));

    register #($bits(data_store_E)) data_store_M_REG(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(data_store_E),
            .Q(data_store));

    register #($bits(data_store_mask_E)) data_store_mask_M_REG(.clk, .rst_l, .en(~halted), .clear(flush_M), .D(data_store_mask_E),
            .Q(data_store_mask));

    //data loading logic and masking
    dataLoad dLoad (.*);


   
    rd_dataLogic rd_dataLog_WAY_0 (.ctrl_signals_M(ctrl_signals_M_WAY_0), .mult_out_M(mult_out_M_WAY_0), 
                                   .alu_out_M(alu_out_M_WAY_0), 
                                   .pc_M(pc_M_WAY_0), .rd_data_M(rd_data_M_WAY_0), .data_res_M);

    rd_dataLogic rd_dataLog_WAY_1 (.ctrl_signals_M(ctrl_signals_M_WAY_1), .mult_out_M(mult_out_M_WAY_1), 
                                   .alu_out_M(alu_out_M_WAY_1), 
                                   .pc_M(pc_M_WAY_1), .rd_data_M(rd_data_M_WAY_1), .data_res_M);

    register #($bits(rd_data_M_WAY_0)) rd_data_W_WAY_0_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rd_data_M_WAY_0),
            .Q(rd_data_W_WAY_0));

    register #($bits(rd_data_M_WAY_1)) rd_data_W_WAY_1_REG(.clk, .rst_l, .en(~halted), .clear(1'b0), .D(rd_data_M_WAY_1),
            .Q(rd_data_W_WAY_1));
    
    
   
    /* Writeback data to the register file, and handle any syscalls/exceptions.
     * Note that you don't need to support exceptions, they are here simply to
     * aid with debugging.
    */
    logic           syscall_halt_WAY_0, syscall_halt_WAY_1, exception_halt, exception_halt0, exception_halt1, syscall_halt, no_op;
    //check this
    assign no_op = pc_W_WAY_1 < 32'b1;
    assign a0_value_WAY_0         = rs1_data_final_W_WAY_0;
    assign a0_value_WAY_1         = rs1_data_final_W_WAY_1;
    assign syscall_halt_WAY_0     = (ctrl_signals_W_WAY_0.syscall && (a0_value_WAY_0 == ECALL_ARG_HALT));
    assign syscall_halt_WAY_1     = (ctrl_signals_W_WAY_1.syscall && (a0_value_WAY_1 == ECALL_ARG_HALT));
    assign syscall_halt = syscall_halt_WAY_0 || syscall_halt_WAY_1;
    assign exception_halt0   = instr_mem_excpt | data_mem_excpt | ctrl_signals_W_WAY_0.illegal_instr;
    assign exception_halt1 = (ctrl_signals_W_WAY_1.illegal_instr & (~no_op));
    assign exception_halt = exception_halt0 || exception_halt1;
    assign halted = rst_l & (syscall_halt_WAY_0 | exception_halt | (syscall_halt_WAY_1 && should_halt));
`ifdef SIMULATION_18447
    always_ff @(posedge clk) begin
        if (rst_l && instr_mem_excpt) begin
            $display("Instruction memory exception at address 0x%08x.", instr_addr << 2);
        end
        if (rst_l && data_mem_excpt) begin
            $display("Data memory exception at address 0x%08x.", data_addr << 2);
        end
        if (rst_l && syscall_halt) begin
            $display("ECALL invoked with halt argument. Terminating simulation at 0x%08x.", pc_F);
        end
    end
`endif /* SIMULATION_18447 */

    /* When the design is compiled for simulation, the Makefile defines
     * SIMULATION_18447. You can use this to have code that is there for
     * simulation, but is discarded when the design is synthesized. Useful
     * for constructs that can't be synthesized. */
`ifdef SIMULATION_18447
`ifdef TRACE

    
    logic [31:0] cycle_count, instr_count, prev_instr_D_WAY_0, instr_E_count, prev_instr_E_WAY_0;
    logic [31:0] ID_0_Instr, ID_1_Instr, ID_2_Instr, Mem_Con;
    logic [31:0] ID_1_Data_Hazard_WAY_1, ID_1_single, ID_0_data_hazard; 
    logic [31:0] ID_0_Invalid, control_executed_count, mispredict_count;
    
    always_ff @(posedge clk) begin
      if(~rst_l) begin
        cycle_count   <= 0;
        instr_count   <= 0;
        instr_E_count <= 0;
        ID_0_Instr <= 0;
        ID_1_Instr <= 0;
        ID_2_Instr <= 0;
        Mem_Con <= 0;
        ID_1_Data_Hazard_WAY_1 <= 0;
        ID_1_single <= 0;
        ID_0_Invalid <= 0;
        control_executed_count <= 0;
        mispredict_count <= 0;
        ID_0_data_hazard <= 0;
        /*
        stall_count_D <= 0;
        stalled_0     <= 0;
        stalled_1     <= 0;
        stalled_2     <= 0;
        stalled_3     <= 0;
        prev_instr_D_WAY_0 <= instr_D_WAY_0;
        prev_stalled_0 <= 0;
        prev_stalled_1 <= 0;
        prev_stalled_2 <= 0;
        */
      end

      else begin
        cycle_count <= cycle_count + 1;
        if(instr_D_WAY_0 != 32'd0 && prev_instr_D_WAY_0 != instr_D_WAY_0) begin
           if (ctrl_signals_D_WAY_0.syscall) instr_count <= instr_count + 1;
           else instr_count <= instr_count + 2;
        end
    
        if(instr_E_WAY_0 != 32'd0 && prev_instr_E_WAY_0 != instr_E_WAY_0) begin
           if (ctrl_signals_E_WAY_0.syscall) instr_E_count <= instr_E_count + 1;
           else instr_E_count <= instr_E_count + 2;
        end
    
        prev_instr_D_WAY_0 <= instr_D_WAY_0;
        prev_instr_E_WAY_0 <= instr_E_WAY_0;


        if(stall_WAY_0 && stall_WAY_1)   ID_0_Instr <= ID_0_Instr + 1;
        if(stall_WAY_0 && ~stall_WAY_1)  ID_1_Instr <= ID_1_Instr + 1;
        if(~stall_WAY_0 && stall_WAY_1)  ID_1_Instr <= ID_1_Instr + 1;
        if(~stall_WAY_0 && ~stall_WAY_1) ID_2_Instr <= ID_2_Instr + 1; 
        if(mem_concur) Mem_Con <= Mem_Con + 1;
        if (concur) ID_1_Data_Hazard_WAY_1 <= ID_1_Data_Hazard_WAY_1 + 1;
        if(ctrl_signals_D_WAY_1.illegal_instr || ctrl_signals_D_WAY_0.syscall ||
           (ctrl_signals_D_WAY_0.pc_source == PC_uncond && predicted_WAY_0_D) ||
           (predicted_WAY_0_D && predicted_bcond_D)
          ) ID_1_single <= ID_1_single + 1;
        if (ctrl_signals_D_WAY_1.illegal_instr && ctrl_signals_D_WAY_0.illegal_instr) ID_0_Invalid <= ID_0_Invalid + 1;
        if ((ctrl_signals_E_WAY_0.pc_source == PC_uncond || ctrl_signals_E_WAY_0.pc_source == PC_cond) && 
            (flush_M || (ctrl_signals_E_WAY_1.pc_source != PC_uncond || ctrl_signals_E_WAY_1.pc_source != PC_cond))) 
            control_executed_count <= control_executed_count + 1;
        if ((ctrl_signals_E_WAY_0.pc_source == PC_uncond || ctrl_signals_E_WAY_0.pc_source == PC_cond) && 
            (~flush_M && (ctrl_signals_E_WAY_1.pc_source == PC_uncond || ctrl_signals_E_WAY_1.pc_source == PC_cond))) 
            control_executed_count <= control_executed_count + 2;
        if (flush) mispredict_count <= mispredict_count + 1;
        if (stall_case_1) ID_0_data_hazard <= ID_0_data_hazard + 1;
        /*
        if(stall) stall_count_D <= stall_count_D + 1;

        if (instr_D != 0) begin
          if(stall & ~prev_stalled_1 & ~prev_stalled_2) prev_stalled_1 <= 1;
          else if(stall & prev_stalled_1) begin prev_stalled_2 <= 1; prev_stalled_1 <= 0; end
          else if(stall & prev_stalled_2) begin stalled_3 <= stalled_3 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & prev_stalled_1) begin stalled_1 <= stalled_1 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & prev_stalled_2) begin stalled_2 <= stalled_2 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & ~prev_stalled_1 & ~prev_stalled_2 & (prev_instr_F != instr)) 
                 begin stalled_0 <= stalled_0 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
        end
        */
      end
    end

    always_ff @(posedge clk) begin
      if(halted) begin
        $display("Cycle Count : %d", cycle_count);
        $display("Instr Count : %d", instr_count);
        $display("Instr Executed : %d", instr_E_count);
        $display("ID_2_instr : %d", ID_2_Instr);
        $display("ID_1_instr : %d", ID_1_Instr);
        $display("ID_0_instr : %d", ID_0_Instr);
        $display("Memory concurrencies(NON-ALU) : %d", Mem_Con);
        $display("ID_1_Data_Hazard_WAY_1 : %d", ID_1_Data_Hazard_WAY_1);
        $display("ID_1_single : %d", ID_1_single);
        $display("ID_0_data_hazard : %d", ID_0_data_hazard);
        $display("ID_0_Invalid : %d", ID_0_Invalid);
        $display("control_executed_count : %d", control_executed_count);
        $display("mispredict_count : %d", mispredict_count);
      end
    end

    //logic [31:0] instr_executed, alu_executed, store_executed, load_executed;
    /*
    logic [31:0] BFTHR_executed, BFTHN_executed, BFTMR_executed, BFTMN_executed, BFNHR_executed, BFNHN_executed, BFNMR_executed, BFNMN_executed;
    logic [31:0] BBTHR_executed, BBTHN_executed, BBTMR_executed, BBTMN_executed, BBNHR_executed, BBNHN_executed, BBNMR_executed, BBNMN_executed;
    logic [31:0] JYHR, JYHN, JYMR, JYMN, JNHR, JNHN, JNMR, JNMN;
    logic [31:0] JR_YHR, JR_YHN, JR_YMR, JR_YMN, JR_NHR, JR_NHN, JR_NMR, JR_NMN;
    logic [31:0] stall_count_D, stalled_0, stalled_1, stalled_2, stalled_3;
    logic [31:0] prev_instr_F, prev_instr_E, prev_stalled_0, prev_stalled_1, prev_stalled_2;

    always_ff @(posedge clk) begin
      if(~rst_l) begin
        cycle_count   <= 0;
        instr_count   <= 0;
        instr_executed <= 0;
        alu_executed <= 0;
        BFTHR_executed <= 0;
        BFTHN_executed <= 0;
        BFTMR_executed <= 0;
        BFTMN_executed <= 0;
        BFNHR_executed <= 0;
        BFNHN_executed <= 0;
        BFNMR_executed <= 0;
        BFNMN_executed <= 0;
        BBTHR_executed <= 0;
        BBTHN_executed <= 0;
        BBTMR_executed <= 0;
        BBTMN_executed <= 0;
        BBNHR_executed <= 0;
        BBNHN_executed <= 0;
        BBNMR_executed <= 0;
        BBNMN_executed <= 0;
        JYHR <= 0;
        JYHN <= 0;
        JYMR <= 0;
        JYMN <= 0;
        JNHR <= 0;
        JNHN <= 0;
        JNMR <= 0;
        JNMN <= 0;
        JR_YHR <= 0;
        JR_YHN <= 0;
        JR_YMR <= 0;
        JR_YMN <= 0;
        JR_NHR <= 0;
        JR_NHN <= 0;
        JR_NMR <= 0;
        JR_NMN <= 0;
        store_executed <= 0;
        load_executed <= 0;
        stall_count_D <= 0;
        stalled_0     <= 0;
        stalled_1     <= 0;
        stalled_2     <= 0;
        stalled_3     <= 0;
        prev_instr_F <= instr;
        prev_instr_E <= instr_E;
        prev_stalled_0 <= 0;
        prev_stalled_1 <= 0;
        prev_stalled_2 <= 0;
      end
      else begin
        cycle_count <= cycle_count + 1;
        if(prev_instr_F != instr) instr_count <= instr_count + 1;
        if((prev_instr_E != instr_E) && (instr_E != 0)) begin
          instr_executed <= instr_executed + 1;

          if(ctrl_signals_E.pc_source == PC_plus4) begin
            if(!ctrl_signals_E.memWrite & !ctrl_signals_E.memRead) alu_executed <= alu_executed + 1;
            else if(ctrl_signals_E.memWrite & !ctrl_signals_E.memRead) store_executed <= store_executed + 1;
            else if(!ctrl_signals_E.memWrite & ctrl_signals_E.memRead) load_executed <= load_executed + 1;
          end

          else if(ctrl_signals_E.pc_source == PC_cond) begin
            if      (~boffset_E[31] && bcond && (tagPC_E == pc_E) && flush) BFTHR_executed <= BFTHR_executed + 1;
            else if (~boffset_E[31] && bcond && (tagPC_E == pc_E) && ~flush) BFTHN_executed <= BFTHN_executed + 1;
            else if (~boffset_E[31] && bcond && (tagPC_E != pc_E) && flush) BFTMR_executed <= BFTMR_executed + 1;
            else if (~boffset_E[31] && bcond && (tagPC_E != pc_E) && ~flush) BFTMN_executed <= BFTMN_executed + 1;
            else if (~boffset_E[31] && ~bcond && (tagPC_E == pc_E) && flush) BFNHR_executed <= BFNHR_executed + 1;
            else if (~boffset_E[31] && ~bcond && (tagPC_E == pc_E) && ~flush) BFNHN_executed <= BFNHN_executed + 1;
            else if (~boffset_E[31] && ~bcond && (tagPC_E != pc_E) && flush) BFNMR_executed <= BFNMR_executed + 1;
            else if (~boffset_E[31] && ~bcond && (tagPC_E != pc_E) && ~flush) BFNMN_executed <= BFNMN_executed + 1;
            else if (boffset_E[31] && bcond && (tagPC_E == pc_E) && flush) BBTHR_executed <= BBTHR_executed + 1;
            else if (boffset_E[31] && bcond && (tagPC_E == pc_E) && ~flush) BBTHN_executed <= BBTHN_executed + 1;
            else if (boffset_E[31] && bcond && (tagPC_E != pc_E) && flush) BBTMR_executed <= BBTMR_executed + 1;
            else if (boffset_E[31] && bcond && (tagPC_E != pc_E) && ~flush) BBTMN_executed <= BBTMN_executed + 1;
            else if (boffset_E[31] && ~bcond && (tagPC_E == pc_E) && flush) BBNHR_executed <= BBNHR_executed + 1;
            else if (boffset_E[31] && ~bcond && (tagPC_E == pc_E) && ~flush) BBNHN_executed <= BBNHN_executed + 1;
            else if (boffset_E[31] && ~bcond && (tagPC_E != pc_E) && flush) BBNMR_executed <= BBNMR_executed + 1;
            else if (boffset_E[31] && ~bcond && (tagPC_E != pc_E) && ~flush) BBNMN_executed <= BBNMN_executed + 1;       
          end
          
          else if(ctrl_signals_E.pc_source == PC_uncond) begin
            if(ctrl_signals_E.alu_op == ALU_JAL) begin
              if     ((rd_E == X1) && (tagPC_E == pc_E) && flush)   JYHR <= JYHR + 1;
              else if((rd_E == X1) && (tagPC_E == pc_E) && ~flush)  JYHN <= JYHN + 1;
              else if((rd_E == X1) && (tagPC_E != pc_E) && flush)   JYMR <= JYMR + 1;
              else if((rd_E == X1) && (tagPC_E != pc_E) && ~flush)  JYMN <= JYMN + 1;
              else if((rd_E != X1) && (tagPC_E == pc_E) && flush)   JNHR <= JNHR + 1;
              else if((rd_E != X1) && (tagPC_E == pc_E) && ~flush)  JNHN <= JNHN + 1;
              else if((rd_E != X1) && (tagPC_E != pc_E) && flush)   JNMR <= JNMR + 1;
              else if((rd_E != X1) && (tagPC_E != pc_E) && ~flush)  JNMN <= JNMN + 1;
            end
            else begin
              if     ((rs1_E == X1) && (tagPC_E == pc_E) && flush)   JR_YHR <= JR_YHR + 1;
              else if((rs1_E == X1) && (tagPC_E == pc_E) && ~flush)  JR_YHN <= JR_YHN + 1;
              else if((rs1_E == X1) && (tagPC_E != pc_E) && flush)   JR_YMR <= JR_YMR + 1;
              else if((rs1_E == X1) && (tagPC_E != pc_E) && ~flush)  JR_YMN <= JR_YMN + 1;
              else if((rs1_E != X1) && (tagPC_E == pc_E) && flush)   JR_NHR <= JR_NHR + 1;
              else if((rs1_E != X1) && (tagPC_E == pc_E) && ~flush)  JR_NHN <= JR_NHN + 1;
              else if((rs1_E != X1) && (tagPC_E != pc_E) && flush)   JR_NMR <= JR_NMR + 1;
              else if((rs1_E != X1) && (tagPC_E != pc_E) && ~flush)  JR_NMN <= JR_NMN + 1;
            end
          end
        end
        prev_instr_F <= instr;
        prev_instr_E <= instr_E;
        if(stall) stall_count_D <= stall_count_D + 1;

        if (instr_D != 0) begin
          if(stall & ~prev_stalled_1 & ~prev_stalled_2) prev_stalled_1 <= 1;
          else if(stall & prev_stalled_1) begin prev_stalled_2 <= 1; prev_stalled_1 <= 0; end
          else if(stall & prev_stalled_2) begin stalled_3 <= stalled_3 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & prev_stalled_1) begin stalled_1 <= stalled_1 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & prev_stalled_2) begin stalled_2 <= stalled_2 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
          else if(~stall & ~prev_stalled_1 & ~prev_stalled_2 & (prev_instr_F != instr)) 
                 begin stalled_0 <= stalled_0 + 1; prev_stalled_1 <= 0; prev_stalled_2 <= 0; end
        end

        else begin
          stalled_0      <= 0;
          stalled_1      <= 0;
          stalled_2      <= 0;
          stalled_3      <= 0;
          prev_stalled_0 <= 0;
          prev_stalled_1 <= 0;
          prev_stalled_2 <= 0;
        end
        
   
      end
    end

    always_ff @(posedge clk) begin
      if(halted) begin
        $display("Cycle Count : %d", cycle_count);
        $display("Instr Count : %d", instr_count);
        $display("Stall Count (Decode Stage) : %d", stall_count_D);
        $display("Instr executed: %d", instr_executed);
        $display("Alu instr executed: %d", alu_executed);
        $display("Store instr executed: %d", store_executed);
        $display("Load instr executed: %d", load_executed);
        $display("BFTHR_executed: %d", BFTHR_executed);
        $display("BFTHN_executed: %d", BFTHN_executed);
        $display("BFTMR_executed: %d", BFTMR_executed);
        $display("BFTMN_executed: %d", BFTMN_executed);
        $display("BFNHR_executed: %d", BFNHR_executed);
        $display("BFNHN_executed: %d", BFNHN_executed);
        $display("BFNMR_executed: %d", BFNMR_executed);
        $display("BFNMN_executed: %d", BFNMN_executed);
        $display("BBTHR_executed: %d", BBTHR_executed);
        $display("BBTHN_executed: %d", BBTHN_executed);
        $display("BBTMR_executed: %d", BBTMR_executed);
        $display("BBTMN_executed: %d", BBTMN_executed);
        $display("BBNHR_executed: %d", BBNHR_executed);
        $display("BBNHN_executed: %d", BBNHN_executed);
        $display("BBNMR_executed: %d", BBNMR_executed);
        $display("BBNMN_executed: %d", BBNMN_executed);

        $display("JYHR: %d", JYHR);
        $display("JYHN: %d", JYHN);
        $display("JYMR: %d", JYMR);
        $display("JYMN: %d", JYMN);
        $display("JNHR: %d", JNHR);
        $display("JNHN: %d", JNHN);
        $display("JNMR: %d", JNMR);
        $display("JNMN: %d", JNMN);

        $display("JR_YHR: %d", JR_YHR);
        $display("JR_YHN: %d", JR_YHN);
        $display("JR_YMR: %d", JR_YMR);
        $display("JR_YMN: %d", JR_YMN);
        $display("JR_NHR: %d", JR_NHR);
        $display("JR_NHN: %d", JR_NHN);
        $display("JR_NMR: %d", JR_NMR);
        $display("JR_NMN: %d", JR_NMN);
      end
    end

   */
    opcode_t opcode;
    funct7_t funct7;
    rtype_funct3_t rtype_funct3;
    itype_int_funct3_t itype_int_funct3;
    assign opcode           = opcode_t'(instr[6:0]);
    assign funct7           = funct7_t'(instr[31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr[14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr[14:12]);

    /* Cycle-by-cycle trace messages. You'll want to comment this out for
     * longer tests, or they will take much, much longer to run. Be sure to
     * comment this out before submitting your code, so tests can be run
     * quickly. */
    
    always_ff @(posedge clk) begin
        if (rst_l) begin
            $display({"\n", {80{"-"}}});
            $display("- Simulation Cycle %0d", $time);
            $display({{80{"-"}}, "\n"});

            $display("\tPC: 0x%08x", pc_F);
            $display("\tInstruction: 0x%08x\n", instr);

            $display("\tInstruction Memory Exception: %0b", instr_mem_excpt);
            $display("\tData Memory Exception: %0b", data_mem_excpt);
            $display("\tIllegal Way 0 Instruction Exception: %0b", ctrl_signals_D_WAY_0.illegal_instr);
            $display("\tIllegal Way 1 Instruction Exception: %0b", ctrl_signals_D_WAY_1.illegal_instr);
            $display("\tHalted: %0b\n", halted);

            $display("\tOpcode: 0x%02x (%s)", opcode, opcode.name);
            $display("\tFunct3: 0x%01x (%s | %s)", rtype_funct3, rtype_funct3.name, itype_int_funct3.name);
            $display("\tFunct7: 0x%02x (%s)", funct7, funct7.name);
            /*
            $display("\trs1_: %0d", rs1_D);
            $display("\trs2: %0d", rs2_D);
            $display("\trd: %0d", rd_D);
            $display("\trs1: %0d", rs1_D);
            $display("\trs2: %0d", rs2_D);
            $display("\trd: %0d", rd_D);
            $display("\tSign Extended Immediate: %0d", se_immediate_E);
            */
        end
    end
   

`endif /* TRACE */
`endif /* SIMULATION_18447 */

endmodule: riscv_core

/**
 * The arithmetic-logic unit (ALU) for the RISC-V processor.
 *
 * The ALU handles executing the current instruction, producing the
 * appropriate output based on the ALU operation specified to it by the
 * decoder. AUIPC, LUI, and JAL instructions included here as well.
 *
 * Inputs:
 *  - alu_src1      The first operand to the ALU.
 *  - alu_src2      The second operand to the ALU.
 *  - pc            The third operand to the ALU           
 *  - alu_op        The ALU operation to perform. 
 * Outputs:
 *  - alu_out       The result of the ALU operation on the two sources.
 **/
module riscv_alu
    (input  logic [31:0]    alu_src1,
     input  logic [31:0]    alu_src2,
     input  logic [31:0]    pc,
     input  alu_op_t        alu_op,
     output logic [31:0]    alu_out);

    logic [31:0]    sum;

    adder #($bits(alu_src1)) ALU_Adder(.A(alu_src1), 
                                       .B((alu_op==ALU_SUB)?(~alu_src2):alu_src2), 
                                       .cin(alu_op==ALU_SUB),
                                       .sum, .cout());

   

    always_comb begin
        unique case (alu_op)
            ALU_ADD: alu_out = sum;
            ALU_SUB: alu_out = sum;
            ALU_LUI: alu_out = alu_src2 << 12;
            ALU_AUIPC: alu_out = pc + (alu_src2 << 12);
            ALU_SLL: alu_out = alu_src1 << (alu_src2 % 32);
            ALU_SLT: begin
              if (alu_src1[31] & alu_src2[31]) alu_out = (alu_src1[30:0] < alu_src2[30:0]) ? 1: 0;
              else if (alu_src1[31] & ~alu_src2[31]) alu_out = 1;
              else if (~alu_src1[31] & alu_src2[31]) alu_out = 0;
              else alu_out = (alu_src1 < alu_src2) ? 1: 0;
            end
            ALU_SLTU: alu_out = (alu_src1 < alu_src2) ? 1: 0;
            ALU_XOR: alu_out = alu_src1 ^ alu_src2;
            ALU_SRL: alu_out = alu_src1 >> (alu_src2 % 32);
            ALU_SRA: begin
              if(alu_src1[31]) alu_out = ( alu_src1 >> (alu_src2 % 32) ) | (32'hFFFFFFFF << (32 - (alu_src2 % 32) ) );
              else alu_out = alu_src1 >> (alu_src2 % 32);
            end
            ALU_OR: alu_out = alu_src1 | alu_src2;
            ALU_AND: alu_out = alu_src1 & alu_src2;
            ALU_JAL: alu_out = pc + alu_src2;
            default: alu_out = 'bx;
        endcase
        
    end

endmodule: riscv_alu
