`default_nettype none

module always_not_taken(
  input  logic taken, en, 
  input  logic [1:0] history,
  output logic [1:0] updated_history,
  output logic take_branch);

  assign take_branch = 0;
  assign updated_history = 2'b0;
 
endmodule: always_not_taken


module one_bit_counter(
  input  logic taken, en, 
  input  logic [1:0] history,
  output logic [1:0] updated_history,
  output logic take_branch);

  always_comb begin
    unique case(history) 
      2'b00: begin
        take_branch = 0;
        if(en) begin 
          if(taken) updated_history = 2'b11;
          else updated_history = 2'b00;
        end
        else updated_history = history;
      end

      2'b11: begin
        take_branch = 1;
        if(en) begin
          if(taken) updated_history = 2'b11;
          else updated_history = 2'b00;
        end
        else updated_history = history;
      end

      default: begin
        $display("Default 2bit counter\n");
        take_branch = 0;
        updated_history = 2'b00;
      end
          
    endcase
  end

endmodule: one_bit_counter

module two_bit_hysteresis_counter(
  input logic taken, en,
  input logic [1:0] history,
  output logic [1:0] updated_history,
  output logic take_branch);
  
  /* 00 : Weak Not Taken
     01 : Strong Not Taken
     11 : Weak Taken
     10 : Strong Taken 
  */
  always_comb begin
    unique case(history) 
      2'b00: begin
        take_branch = 0;
        if (en) begin
          if(taken) updated_history = 2'b10;
          else updated_history = 2'b01;
        end
        else begin
          updated_history = history;
        end
      end

      2'b01: begin
        take_branch = 0;
        if (en) begin
          if(taken) updated_history = 2'b00;
          else updated_history = 2'b01;
        end
        else begin
          updated_history = history;
        end
      end

      2'b11: begin
        take_branch = 1;
        if (en) begin
          if(taken) updated_history = 2'b10;
          else updated_history = 2'b01;
        end
        else begin
          updated_history = history;
        end
      end
         
      2'b10: begin
        take_branch = 1;
        if (en) begin
          if(taken) updated_history = 2'b10;
          else updated_history = 2'b11;
        end
        else begin
          updated_history = history;
        end
      end 

      default: begin
        $display("Default 2bit counter\n");
        take_branch = 0;
        updated_history = 2'b00;
      end

    endcase
  end

endmodule: two_bit_hysteresis_counter