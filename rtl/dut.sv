//---------------------------------------------------------------------------
// DUT - 564/464 Project - RATAN GUNDAMI
//---------------------------------------------------------------------------
`include "common.vh"

module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//input SRAM interface
  output wire                           dut__tb__sram_input_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_input_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_read_address  , 
  input  wire signed [`SRAM_DATA_RANGE     ]   tb__dut__sram_input_read_data     ,     

//weight SRAM interface
  output wire                           dut__tb__sram_weight_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_weight_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_read_address  , 
  input  wire signed [`SRAM_DATA_RANGE     ]   tb__dut__sram_weight_read_data     ,     

//result SRAM interface
  output wire                           dut__tb__sram_result_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_write_address ,
  output wire signed [`SRAM_DATA_RANGE     ]   dut__tb__sram_result_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_read_address  , 
  input  wire signed [`SRAM_DATA_RANGE     ]   tb__dut__sram_result_read_data     ,     

//scratchpad SRAM interface
  output wire                           dut__tb__sram_scratchpad_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_scratchpad_write_address ,
  output wire signed [`SRAM_DATA_RANGE     ]   dut__tb__sram_scratchpad_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_scratchpad_read_address  , 
  input  wire signed [`SRAM_DATA_RANGE     ]   tb__dut__sram_scratchpad_read_data  

);

typedef enum bit[2:0] {
  IDLE              = 3'd0, 
  INITIAL           = 3'd1,   
  READ_ADDRESS      = 3'd2,
  MATRIX_COMPLETED  = 3'd3} states;

states current_state, next_state; 

// Matrix variables

logic signed [`SRAM_DATA_RANGE     ]   A_matrix_read_data;
logic signed [`SRAM_DATA_RANGE     ]   B_matrix_read_data;

logic [`SRAM_ADDR_RANGE     ]   A_matrix_read_addr;
logic [`SRAM_ADDR_RANGE     ]   B_matrix_read_addr;
logic [`SRAM_ADDR_RANGE     ]   C_matrix_write_addr;


// Matrix Registers to initialize inside FF
logic [`SRAM_ADDR_RANGE] dut__tb__sram_scratchpad_write_address_r ;

// Control Signals
logic dut_ready_r;
logic write_flag;
logic reset_transpose;
logic matrix_change_flag;
logic [1:0] initial_flag;

// Matrix
logic [15:0] rows_A; 
logic [15:0] columns_A; 
logic [15:0] rows_B; 
logic [15:0] columns_B; 

// counters
logic [15:0] A_row_counter; 
logic [15:0] A_column_counter; 
logic [3:0] matrix_mult_count;
logic [3:0] delayed_matrix_change;
logic [15:0] result_counter; 
logic [15:0] write_counter;  

// reg [`SRAM_DATA_RANGE] A_matrix_size;
// logic [15:0] B_matrix_size;
// logic [15:0] result_matrix_size;
logic [15:0] QKV_matrix_size;

// Local control path variables
logic signed [`SRAM_DATA_RANGE     ] accum_result_r;
logic signed [`SRAM_DATA_RANGE     ] mult_res;

// -------- Logic for FSM FF -----
always_ff @(posedge clk) begin
  if (!reset_n)
    current_state <= IDLE;
  else
    current_state <= next_state;
end

// ----------- Contains both Next state logic as well as Output logic ----------
always_comb begin
  case (current_state) // synopsys full_case parallel_case
    IDLE: begin
      dut_ready_r <= 1; 
      if (dut_valid==1) begin
        next_state <= INITIAL;
        // $display("Time: %0t | State: IDLE -> INITIAL", $time);
      end
      else begin
        next_state <= IDLE;
        // $display("Time: %0t | State: IDLE remains IDLE", $time);
      end
    end

    INITIAL: begin
      dut_ready_r <= 0; 
      next_state <= READ_ADDRESS;
      // $display("Time: %0t | State: INITIAL -> READ", $time);
    end

    READ_ADDRESS: begin
      dut_ready_r <= 0;
      if ((write_counter == ((rows_A*columns_B)-1)) && (A_column_counter == columns_A)) begin
        next_state <= MATRIX_COMPLETED;
      end
      else begin
        next_state <= READ_ADDRESS;
      end
    end

    MATRIX_COMPLETED: begin
      dut_ready_r <= 0; 
      if (matrix_mult_count == 4) begin // Scalar Dot Product is calculated, Go to IDLE
        next_state <= IDLE;
      end
      else if (matrix_mult_count > 1) begin  // it has almost finished calculating V matrix 
        next_state <= INITIAL;
      end
      else begin
        next_state <= READ_ADDRESS;
      end
    end

    default: begin
      dut_ready_r <= 0; 
      next_state <= IDLE;
      // $display("Time: %0t | State: DEFAULT (unexpected) -> IDLE", $time);
    end
  endcase
end

assign dut_ready = dut_ready_r;


// Calculate rows and columns - INITIAL STATE
always_ff @(posedge clk) begin
  if(current_state == IDLE) begin
    rows_A <= 16'b0;
    columns_A <= 16'b0;
    rows_B <= 16'b0;
    columns_B <= 16'b0; 
  end 
  else if (current_state == INITIAL) begin
    if (matrix_mult_count == 3) begin
      rows_A <= rows_A; // 2, 4
      columns_A <= columns_B; // 4, 16
      rows_B <= columns_B; // 4, 16
      columns_B <= rows_A; // 2, 4
      // B_matrix_size <= columns_B*rows_A; // 4x2 
      // result_matrix_size <= rows_A*rows_A; // 2x2
    end
    else if (matrix_mult_count == 4) begin
      rows_A <= rows_A; // 2, 4
      columns_A <= columns_B; // 2, 4
      rows_B <= rows_A; // 2, 4
      columns_B <= columns_A; // 4, 16
      // B_matrix_size <= rows_A*columns_A; 
      // result_matrix_size <= rows_A*columns_A; 
    end
    else begin
      rows_A <= A_matrix_read_data[31:16]; // , 4
      columns_A <= A_matrix_read_data[15:0]; // , 16
      rows_B <= B_matrix_read_data[31:16]; // , 16
      columns_B <= B_matrix_read_data[15:0]; // , 16
      // B_matrix_size <= B_matrix_read_data[31:16]*B_matrix_read_data[15:0]; 
      // result_matrix_size <= A_matrix_read_data[31:16]*B_matrix_read_data[15:0];    
      // $display("Time: %0t | State: INITIAL | calculated rows and columns", $time);
    end
  end
end

// Sequential Logic to calculate read address
always_ff @(posedge clk) begin
  if(current_state == IDLE) begin
    A_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;
    B_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;

    matrix_mult_count <= 0;
    A_row_counter <= 0;
    A_column_counter <= 0;
    result_counter <= 0;
    matrix_change_flag <= 0;
  end

  else if(current_state == READ_ADDRESS && next_state != MATRIX_COMPLETED) begin
  matrix_change_flag <= 0;
    // Depending on the matrix count change the address and SRAMs
    if (matrix_change_flag == 1) begin
      case (matrix_mult_count)
        1: // Executes after Q is finished
        begin 
          A_matrix_read_addr <= `SRAM_ADDR_WIDTH'b1;
          B_matrix_read_addr <= (rows_B*columns_B) + `SRAM_ADDR_WIDTH'b1;
          QKV_matrix_size <= (rows_A*columns_B);   
        end     
        2: // Executes after K is finished
        begin
          A_matrix_read_addr <= `SRAM_ADDR_WIDTH'b1;
          B_matrix_read_addr <= (rows_B*columns_B) + (rows_B*columns_B) + `SRAM_ADDR_WIDTH'b1; 
        end
        3: // Executes after V is finished
        begin
          A_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;
          B_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;
        end
        4: // Executes after S is finished
        begin
          A_matrix_read_addr <= QKV_matrix_size + QKV_matrix_size + QKV_matrix_size;
          B_matrix_read_addr <= QKV_matrix_size; 
        end
        default: // Executes if no values match
        begin
          A_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;
          B_matrix_read_addr <= `SRAM_ADDR_WIDTH'b0;
        end
      endcase

      // Update the counters as required
      A_column_counter <= A_column_counter + 1;
    end

    else begin
      // Change the row
      if((A_column_counter == columns_A) && (result_counter == (columns_B-1))) begin 
        // Calculate next address
        A_matrix_read_addr <= A_matrix_read_addr + `SRAM_ADDR_WIDTH'b1;
        B_matrix_read_addr <= (B_matrix_read_addr - (rows_B*columns_B)) + `SRAM_ADDR_WIDTH'b1;
        // $display("Time: %0t | State: INITIAL | B_matrix_read_addr : %0d, %0d, %0d", $time, B_matrix_read_addr, B_matrix_size, result_counter + 1);
        // Update the counters as required
        result_counter <= 0;
        A_column_counter <= 1;
        A_row_counter <= A_row_counter + 1;
      end

      // Reset the A row to first column of current row
      else if ((A_column_counter == columns_A)) begin
        // Calculate next address
        A_matrix_read_addr <= A_matrix_read_addr - columns_A + `SRAM_ADDR_WIDTH'b1;
        B_matrix_read_addr <= B_matrix_read_addr + `SRAM_ADDR_WIDTH'b1;

        // Update the counters as required
        A_column_counter <= 1;
        result_counter <= result_counter + 1;  // Update result_counter every time write is done
      end
      else begin
        // Calculate next address
        A_matrix_read_addr <= A_matrix_read_addr + `SRAM_ADDR_WIDTH'b1;
        B_matrix_read_addr <= B_matrix_read_addr + `SRAM_ADDR_WIDTH'b1;

        // Update the counters as required
        A_column_counter <= A_column_counter + 1;
      end
    end
  end

  else if(current_state == MATRIX_COMPLETED) begin
      // Re initialize all the variables
      A_column_counter <= 0;
      result_counter <= 0;
      A_row_counter <= 0;    
      matrix_change_flag <= 1;

      // Increment the matrix completed counter  
      matrix_mult_count <= matrix_mult_count+1;
      // $display("Time: %0t | State: MATRIX_COMPLETED", $time);
  end
end

// Write Logic
always_ff @(posedge clk) begin
  if(current_state == IDLE) begin
    C_matrix_write_addr <= `SRAM_ADDR_WIDTH'b0;
    dut__tb__sram_scratchpad_write_address_r <= `SRAM_ADDR_WIDTH'b0;

    write_flag <= 0;
    reset_transpose <= 0;
    write_counter <= 0;
  end

  if(current_state == INITIAL) begin
    write_flag <= 0;
    write_counter <= 0;
  end

  else if(current_state == READ_ADDRESS) begin
    // Depending on the matrix count change the address and SRAMs
    if (matrix_change_flag == 1) begin
      write_flag <= 1'b0;
      reset_transpose <= 1;
    end

    else begin
      // Change the row
      if((A_column_counter == columns_A) && (result_counter == (columns_B-1))) begin 
        // Enable the write flag
        write_flag <= 1'b1;
        write_counter <= write_counter + 1'b1;
        C_matrix_write_addr <= C_matrix_write_addr + 1;
        reset_transpose <= 1;

        // Check if write to Scratchpad (K and V matrices)
        if(matrix_mult_count == 1) 
        begin
          dut__tb__sram_scratchpad_write_address_r <= dut__tb__sram_scratchpad_write_address_r + `SRAM_ADDR_WIDTH'b1;
        end
        else if (matrix_mult_count == 2)
        begin
          dut__tb__sram_scratchpad_write_address_r <= dut__tb__sram_scratchpad_write_address_r + rows_A;
        end
      end

      // Reset the A row to first column of current row
      else if ((A_column_counter == columns_A)) begin      
        // Enable the write flag
        write_flag <= 1'b1;
        write_counter <= write_counter + 1'b1;
        C_matrix_write_addr <= C_matrix_write_addr + 1;

        // Check if write to Scratchpad (K and V matrices)
        if(matrix_mult_count == 1) 
        begin
          dut__tb__sram_scratchpad_write_address_r <= dut__tb__sram_scratchpad_write_address_r + `SRAM_ADDR_WIDTH'b1;
        end
        else if (matrix_mult_count == 2)
        begin
          if(reset_transpose == 1) 
          begin 
            dut__tb__sram_scratchpad_write_address_r <= (rows_A*columns_B) + A_row_counter + 1;
            reset_transpose <= 0;
          end
          else
          begin
            dut__tb__sram_scratchpad_write_address_r <= dut__tb__sram_scratchpad_write_address_r + rows_A;
          end
        end
      end
      else begin
        write_flag <= 1'b0;
      end
    end
  end
  else if(current_state == MATRIX_COMPLETED) begin
    write_flag <= 1'b0;
    write_counter <= 0;
  end
end


// Handle Initial Stage Delay
always_ff @(posedge clk) begin
  if(current_state == IDLE || current_state == INITIAL) begin
    accum_result_r <= `SRAM_DATA_WIDTH'b0;  
    initial_flag <= 2;
  end
  else if(initial_flag > 0) begin
    initial_flag <= initial_flag - 1;
    accum_result_r <= `SRAM_DATA_WIDTH'b0; 
  end
  else if(write_flag == 1'b1) begin
    accum_result_r <= `SRAM_DATA_WIDTH'b0; 
  end
  else if(matrix_change_flag == 1'b1) begin
    accum_result_r <= `SRAM_DATA_WIDTH'b0; 
    initial_flag <= 1;
  end  
  else begin
    accum_result_r <= accum_result_r + mult_res;    
  end
end


// logic to change the matrix
always_ff @(posedge clk) begin
  if(matrix_mult_count > 0) begin
    delayed_matrix_change <= matrix_mult_count;  
  end
  else begin
    delayed_matrix_change <= 0;  
  end
end


// assign for multiplication
assign mult_res = A_matrix_read_data*B_matrix_read_data;

// assign statements for input matrix
assign dut__tb__sram_input_write_enable = 1'b0;
assign dut__tb__sram_input_write_address = `SRAM_ADDR_WIDTH'b0;
assign dut__tb__sram_input_write_data = `SRAM_DATA_WIDTH'b0;
assign dut__tb__sram_input_read_address = (delayed_matrix_change < 3) ? A_matrix_read_addr : `SRAM_DATA_WIDTH'b0;

// assign statements for weight matrix
assign dut__tb__sram_weight_write_enable = 1'b0;
assign dut__tb__sram_weight_write_address = `SRAM_ADDR_WIDTH'b0;
assign dut__tb__sram_weight_write_data = `SRAM_DATA_WIDTH'b0;
assign dut__tb__sram_weight_read_address = (delayed_matrix_change < 3) ? B_matrix_read_addr : `SRAM_DATA_WIDTH'b0;

// assign statements for result matrix
assign dut__tb__sram_result_read_address = (delayed_matrix_change < 3) ? `SRAM_DATA_WIDTH'b0 : A_matrix_read_addr;
assign dut__tb__sram_result_write_enable = write_flag;
assign dut__tb__sram_result_write_address = C_matrix_write_addr - 1;
assign dut__tb__sram_result_write_data = accum_result_r + mult_res;

// assign statements for scratchpad matrix
assign dut__tb__sram_scratchpad_read_address = (delayed_matrix_change < 3) ? `SRAM_DATA_WIDTH'b0 : B_matrix_read_addr;
assign dut__tb__sram_scratchpad_write_enable = ((delayed_matrix_change < 1) || (delayed_matrix_change > 2)) ? `SRAM_DATA_WIDTH'b0 : write_flag;
assign dut__tb__sram_scratchpad_write_address = dut__tb__sram_scratchpad_write_address_r - 1;
assign dut__tb__sram_scratchpad_write_data = ((delayed_matrix_change < 1) || (delayed_matrix_change > 2)) ? `SRAM_DATA_WIDTH'b0 : accum_result_r + mult_res;

assign A_matrix_read_data = (delayed_matrix_change < 3) ? tb__dut__sram_input_read_data : tb__dut__sram_result_read_data;
assign B_matrix_read_data = (delayed_matrix_change < 3) ? tb__dut__sram_weight_read_data : tb__dut__sram_scratchpad_read_data;

endmodule
