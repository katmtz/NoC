/*********************************************
 *  18-341 Fall 2015                         *
 *  Project 3                                *
 *  Network-on-Chip router                   *
 *********************************************/
/*
 * A router transfers packets between nodes and other routers.
 */
module router(clk, rst_b,
              free_outbound, put_outbound, payload_outbound,
              free_inbound, put_inbound, payload_inbound);
  parameter ROUTERID = 0; // To differentiate between routers
  input  clk, rst_b;

  // self -> destination (sending a payload)
  input [3:0] free_outbound;
  output [3:0] put_outbound;
  output [3:0][7:0] payload_outbound;

  // source -> self (receiving a payload)
  output [3:0] free_inbound;
  input [3:0] put_inbound;
  input [3:0][7:0] payload_inbound;

  // internal data
  pkt_t [3:0] outbound_pkts;         // from out_buffer, to node
  logic [3:0] outbound_pkts_avail;
  pkt_t [3:0] inbound_pkts;          // from node, to channel_sel
  logic [3:0] inbound_pkts_avail;
  logic [3:0] out_queue_ready;        // tell routing outbuffer couldn't read
  logic [3:0][3:0] ob_pkts_packed_avail;
  pkt_t [3:0][3:0] ob_pkts_packed;
  logic [3:0][3:0] ob_pkts_sorted_avail;
  pkt_t [3:0][3:0] ob_pkts_sorted;
  logic [3:0] req;
  logic [3:0][3:0] read;
  logic [3:0] read_sorted;

  // io buffers
  generate
    for (genvar i = 0; i < 4; i++) begin: io
      io_buffer iob (.clk(clk), .rst_b(rst_b),
                 .payload_in(payload_inbound[i]),
                 .payload_in_avail(put_inbound[i]),
                 .payload_out(payload_outbound[i]),
                 .payload_out_avail(put_outbound[i]),
                 .pkt_in(outbound_pkts[i]),
                 .pkt_in_avail(outbound_pkts_avail[i]),
                 .pkt_out(inbound_pkts[i]),
                 .pkt_out_avail(inbound_pkts_avail[i]),
                 .port_in_q_ready(free_inbound[i]),
                 .port_out_q_ready(out_queue_ready[i]),
                 .node_in_ready(free_outbound[i]),
                 .read_in(req[i]));
      routing rt (.clk(clk), .rst_b(rst_b),
                  .pkt_in(inbound_pkts[i]), .pkt_in_avail(inbound_pkts_avail[i]),
                  .out_read(read_sorted[i]),
                  .req(req[i]),
                  .out_data_avail(ob_pkts_packed_avail[i]),
                  .out_data(ob_pkts_packed[i]));
      handle_pkts sort(.clk(clk), .rst_b(rst_b),
                       .pkts_in(ob_pkts_sorted[i]), .pkts_avail(ob_pkts_sorted_avail[i]),
                       .pkt_out(outbound_pkts[i]), .pkt_out_avail(outbound_pkts_avail[i]),
                       .pkt_accept(read[i]), .busy(out_queue_ready[i]));

      always_comb begin
        ob_pkts_sorted[i] = {ob_pkts_packed[3][i], 
                             ob_pkts_packed[2][i], 
                             ob_pkts_packed[1][i], 
                             ob_pkts_packed[0][i]};
        ob_pkts_sorted_avail[i] = {ob_pkts_packed_avail[3][i],
                                   ob_pkts_packed_avail[2][i],
                                   ob_pkts_packed_avail[1][i],
                                   ob_pkts_packed_avail[0][i]};
        read_sorted[i] = read[0][i] | read[1][i] | read[2][i] | read[3][i];
      end
    end
  endgenerate

endmodule

/*
 * hands packets to node one by one
 */
module handle_pkts(clk, rst_b,
                   pkts_in, pkts_avail,
                   pkt_out, pkt_out_avail,
                   pkt_accept, busy);
  input logic clk, rst_b;
  input logic busy;                    // from outbuffer - currently routing a packet out
  input pkt_t [3:0] pkts_in;
  input logic [3:0] pkts_avail;
  output logic [3:0] pkt_accept;
  output pkt_t pkt_out;
  output logic pkt_out_avail;

  reg [3:0] last_used;
 
  always_comb begin
    if (pkts_avail[0] == 1'b1 && !last_used[0]) begin
      pkt_out = pkts_in[0];
      pkt_out_avail = 1'b1;
      pkt_accept = 4'b0001;
    end else begin
    if (pkts_avail[1] == 1'b1 && !last_used[1]) begin
      pkt_out = pkts_in[1];
      pkt_out_avail = 1'b1;
      pkt_accept = 4'b0010;
    end else begin
    if (pkts_avail[2] == 1'b1 && !last_used[2]) begin
      pkt_out = pkts_in[2];
      pkt_out_avail = 1'b1;
      pkt_accept = 4'b0100;
    end else begin
    if (pkts_avail[3] == 1'b1 && !last_used[3]) begin
      pkt_out = pkts_in[3];
      pkt_out_avail = 1'b1;
      pkt_accept = 4'b1000;
    end else begin
      pkt_out = 0;
      pkt_out_avail = 1'b0;
      pkt_accept = 4'b00;
    end end end end
  end

  always_ff @(posedge clk, posedge rst_b) begin
    if (~rst_b) last_used <= 0;
    else last_used <= pkt_accept;
  end
endmodule

/* **********************************************
 * I/O BUFFERS
 * **********************************************
 */

/*
 * wrapper holding input and output buffers to be connected
 * to handle_pkts, routing, and node
 */
module io_buffer(clk, rst_b,
                 payload_in, payload_in_avail,
                 payload_out, payload_out_avail,
                 pkt_in, pkt_in_avail,
                 pkt_out, pkt_out_avail,
                 port_in_q_ready, node_in_ready, read_in,
                 port_out_q_ready);
  input clk, rst_b,
        pkt_in_avail, payload_in_avail,
        node_in_ready, read_in;
  input pkt_t pkt_in;
  input [7:0] payload_in;
  output port_out_q_ready, port_in_q_ready,
         payload_out_avail, pkt_out_avail;
  output pkt_t pkt_out;
  output [7:0] payload_out;
  
  bit ob_empty, ob_q_full, ib_q_full, node_payload_ready;                       
  in_buffer ib (.clk(clk), .rst_b(rst_b),
                .payload(payload_in), .put(payload_in_avail),
                .req(read_in), .pkt_out(pkt_out), .pkt_out_avail(pkt_out_avail),
                .full(ib_q_full));
  out_buffer ob (.clk(clk), .rst_b(rst_b),
                 .pkt(pkt_in), .pkt_avail(pkt_in_avail),
                 .payload_out(payload_out), .payload_out_avail(payload_out_avail),
                 .full(ob_q_full), .req(node_in_ready));

  assign port_out_q_ready = ~ob_q_full,
         port_in_q_ready = ~ib_q_full;
  
endmodule 

/*
 * Recieves packets from a node, queues them, and passes queue
 * data on to channel_sel (wrapper for node queueing modules)
 */
module in_buffer(clk, rst_b,
                 payload, put, req,
                 pkt_out, pkt_out_avail, full);
  input bit clk, rst_b, put, req;
  input bit [7:0] payload;
  output bit pkt_out_avail, full;
  output pkt_t pkt_out;

  // output from serial to queue
  pkt_t pkt;
  logic pkt_avail;
  logic q_empty, free;

  serialToPkt stp (.*);
  fifo q (.clk(clk), .rst_b(rst_b),
          .data_in(pkt), .we(pkt_avail),
          .re(req), .full(full), .empty(q_empty),
          .data_out(pkt_out));

  assign pkt_out_avail = ~q_empty;

endmodule

/*
 * Recieves packets from routing, queues them, passes them on to node
 */
module out_buffer(clk, rst_b,
                  pkt, pkt_avail, req, 
                  payload_out, payload_out_avail, 
                  full);
  input clk, rst_b, pkt_avail, req;
  input pkt_t pkt;
  output [7:0] payload_out;
  output payload_out_avail, full;

  pkt_t pkt_out;
  bit read_in, q_empty, pkt_out_avail;
  assign pkt_out_avail = ~q_empty;

  fifo q (.clk(clk), .rst_b(rst_b),
          .data_in(pkt), .we(pkt_avail),
          .re(read_in), .full(full), .empty(q_empty),
          .data_out(pkt_out));
  pktToSerial pts (.clk(clk), .rst_b(rst_b),
                   .pkt(pkt_out), .pkt_avail(pkt_out_avail),
                   .payload_out(payload_out),
                   .payload_avail(payload_out_avail),
                   .read_in(read_in), .req(req));
 
endmodule

/* **********************************************
 * ROUTING
 * **********************************************
 */

/*
 * Gets a packet and pushes it and a valid bit to the an array of 4 pkts
 * - register holds pkt until recieved signal?
 */
module routing(clk, rst_b,
               pkt_in, pkt_in_avail, out_read,
               out_data, out_data_avail, req);
  parameter ROUTERID = 0;
  input clk, rst_b, pkt_in_avail;
  input pkt_t pkt_in;
  input out_read;                   // one-hot indication that packet went through
  output req;
  output bit  [3:0] out_data_avail;
  output pkt_t [3:0] out_data;

  logic [2:0] send_node;
  get_send_node #(ROUTERID) (.*);

  // indicate whether the outbound buffer was full when we tried to send the last one
  reg read;
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) read <= 1;
    else        read <= (out_read != 0) ? 1'b1 : 1'b0;
  end

  assign req = read && pkt_in_avail;

  always_comb
    case(send_node)
      0: begin
         out_data = {32'b0, 32'b0, 32'b0, pkt_in};
         out_data_avail = {1'b0, 1'b0, 1'b0, pkt_in_avail};
         end
      1: begin
         out_data = {32'b0, 32'b0, pkt_in, 32'b0};
         out_data_avail = {1'b0, 1'b0, pkt_in_avail, 1'b0};
         end
      2: begin
         out_data = {32'b0, pkt_in, 32'b0, 32'b0};
         out_data_avail = {1'b0, pkt_in_avail, 1'b0, 1'b0};
         end
      3: begin 
         out_data = {pkt_in, 32'b0, 32'b0, 32'b0};
         out_data_avail = {pkt_in_avail, 1'b0, 1'b0, 1'b0};
         end
    endcase

endmodule

/*
 * Takes in a packet, determines which node to send it to.
 */
module get_send_node(clk, rst_b,
               pkt_in, pkt_in_avail,
               send_node);
  parameter ROUTERID = 0;
  input clk, rst_b, pkt_in_avail;
  input pkt_t pkt_in;
  output logic [2:0] send_node;

  logic dest_rtr;
  assign dest_rtr = (pkt_in.destID > 4'd3);
  always_comb begin
    if (pkt_in_avail) begin
      if (ROUTERID == 0)
        case(pkt_in.destID)
          0: send_node = 0;
          1: send_node = 2;
          2: send_node = 3;
          default: send_node = 1;
        endcase
      else
        case(pkt_in.destID)
          3: send_node = 0;
          4: send_node = 1;
          5: send_node = 2;
          default: send_node = 3;
        endcase  
    end
    else
      send_node = 2'b0;
  end

endmodule
