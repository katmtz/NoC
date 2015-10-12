/*********************************************
 *  18-341 Fall 2015                         *
 *  Project 3                                *
 *  Network-on-Chip node                     *
 *********************************************/
module node(clk, rst_b, pkt_in, pkt_in_avail, cQ_full, pkt_out, pkt_out_avail,
            free_outbound, put_outbound, payload_outbound,
            free_inbound, put_inbound, payload_inbound);

  parameter NODEID = 0;
  input clk, rst_b;

  // Interface to TestBench
  input pkt_t pkt_in;
  input pkt_in_avail;
  output cQ_full;
  output pkt_t pkt_out;
  output logic pkt_out_avail;

  // Endpoint -> Router transaction
  input free_outbound; // Router -> Endpoint
  output put_outbound; // Endpoint -> Router
  output [7:0] payload_outbound;

  // Router -> Endpoint transaction
  output free_inbound; // Endpoint -> Router
  input put_inbound; // Router -> Endpoint
  input [7:0] payload_inbound;

  logic [31:0] q_out;
  logic q_empty, q_ready, pts_ready;

  assign q_ready = ~q_empty;

  // TB --> NODE --> ROUTER
  fifo        q       (.clk(clk), .rst_b(rst_b), 
                       .data_in(pkt_in), .we(pkt_in_avail), 
                       .data_out(q_out), .re(pts_ready),
                       .full(cQ_full), .empty(q_empty));
  pktToSerial toRouter(.clk(clk), .rst_b(rst_b), 
                       .pkt(q_out), .pkt_avail(q_ready), 
                       .payload_out(payload_outbound), .payload_avail(put_outbound), 
                       .read_in(pts_ready), .req(free_outbound));

  // ROUTER --> NODE --> TB
  serialToPkt toTB    (.clk(clk), .rst_b(rst_b), 
                       .payload(payload_inbound), .put(put_inbound),
                       .pkt(pkt_out), .pkt_avail(pkt_out_avail),
                       .free(free_inbound));
endmodule

module pktToSerial(clk, rst_b, pkt, pkt_avail, payload_out, payload_avail, read_in, req);
  parameter WIDTH = 32;
  input bit clk, rst_b;
  input bit [31:0] pkt;
  input bit pkt_avail, req;
  output bit payload_avail, read_in;
  output bit [7:0] payload_out;

  reg [31:0] buff;
  bit [31:0] buff_in;
  logic [2:0] count;

  pts_fsm fsm (.*);

  always_comb begin
    if (read_in) buff_in = pkt;
    else buff_in = buff;
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) buff <= 0;
    else buff <= buff_in;
  end

  always_comb
    case(count)
      4: payload_out = buff[7:0];
      3: payload_out = buff[15:8];
      2: payload_out = buff[23:16];
      1: payload_out = buff[31:24];
      default: payload_out = 8'hcc;
    endcase

endmodule

// pktToSerial fsm
module pts_fsm(clk, rst_b, pkt_avail, payload_avail, req, read_in, count);
  input bit clk, rst_b, pkt_avail, req;
  output bit payload_avail, read_in;
  output bit [2:0] count;

  enum logic [1:0] {hold = 2'b00, load = 2'b01, send = 2'b10} state, nextState;

  always_comb
    case(state)
      hold: nextState = (pkt_avail) ? load : hold;
      load: nextState = (req) ? send : load;
      send: nextState = (count == 4) ? (pkt_avail) ? load : hold : send;
    endcase
  
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) state <= hold;
    else state <= nextState;
  end

  reg [2:0] count_r;
  logic [2:0] count_in;

  always_comb begin
    if (count == 4) count_in = 0;
    else begin if (state == send || (req && state == load))
      count_in = count_r + 1;
    else
      count_in = 0;
    end
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b)
      count_r <= 0;
    else
      count_r <= count_in;
  end

  logic done;

  assign count = count_r,
         done = (state == send && count == 4),
         payload_avail = (count > 0 && count < 5),
         read_in = (state == hold || done);

endmodule

module serialToPkt(clk, rst_b, payload, put, pkt, pkt_avail, free);
  parameter WIDTH = 32;
  input bit clk, rst_b, put;
  input bit [7:0] payload;
  output logic pkt_avail, free;  
  output logic [31:0] pkt;

  bit [31:0] hold_in;
  bit [1:0] count_in;
  bit free_in;
  
  reg [1:0] count;
  reg [31:0] hold;
  reg free_r;

  assign free_in = ~put; 
  assign count_in = (put) ? count + 1 : 0;
  assign hold_in = (put) ? (payload | (hold << 8)) : 0;

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
        count <= 0;
        free_r <= 1;
    end
    else begin
        count <= count_in;
        hold <= hold_in;
        free_r <= free_in;
    end
  end

  assign pkt = hold;
  assign pkt_avail = (~free && ~put);
  assign free = free_r;

endmodule

/*  Create a fifo (First In First Out) with depth 4 using the given interface
 *  and constraints.
 *  -The fifo is initally empty.
 *  -Reads are combinational, so "data_out" is valid unless "empty" is asserted.
 *   Removal from the queue is processed on the clock edge.
 *  -Writes are processed on the clock edge.  
 *  -If the "we" happens to be asserted while the fifo is full, do NOT update
 *   the fifo.
 *  -Similarly, if the "re" is asserted while the fifo is empty, do NOT update
 *   the fifo. 
 */

module fifo(clk, rst_b, data_in, we, re, full, empty, data_out);
  parameter WIDTH = 32;
  input clk, rst_b;
  input [WIDTH-1:0] data_in;
  input we; //write enable
  input re; //read enable
  output full;
  output empty;
  output bit [WIDTH-1:0] data_out;

  reg [3:0][WIDTH-1:0] Q;
  reg [1:0] w_ptr, r_ptr;
  reg [2:0] count;

  assign full = (count == 3'd4 && ~re),
         empty = (count == 3'd0);
  
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      count <= 0; w_ptr <= 0; r_ptr <= 0; Q <= 0;
    end
    else begin
      if (re && we && count >= 1) begin 
         Q[w_ptr] <= data_in;
         count <= count;
         w_ptr <= w_ptr + 1;
         r_ptr <= r_ptr + 1;
      end else begin
      if (we && !full) begin
        Q[w_ptr] <= data_in;
        count <= count + 1;
        w_ptr <= w_ptr + 1;
      end
      if (re && !empty) begin
        count <= count - 1;
        r_ptr <= r_ptr + 1;
      end
    end
    end
  end  

  assign data_out = Q[r_ptr];

endmodule
