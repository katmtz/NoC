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
  logic [3:0] channel_req;           // request to in_buffer from channel_sel
  logic [3:0] load_out_ready;        // tell routing outbuffer couldn't read

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
                 .port_out_q_ready(load_out_ready[i]),
                 .node_in_ready(free_outbound[i]),
                 .read_in(channel_req[i]));
    end
  endgenerate

  // packet handling and routing:
  // IN_BUFFERS --> CHANNEL_SEL --> ROUTING --> OUT_BUFFERS
  pkt_t pkt_to_route;              // CHANNEL_SEL --> ROUTING
  logic pkt_to_route_avail;         
  logic last_routed;               // ROUTING --> CHANNEL_SEL

  routing rt (.clk(clk), .rst_b(rst_b),
              .pkt_in(pkt_to_route), .pkt_in_avail(pkt_to_route_avail),
              .out_read(load_out_ready),
              .last_routed(last_routed),
              .out_data_avail(outbound_pkts_avail),
              .out_data(outbound_pkts));
  channel_sel cs (.clk(clk), .rst_b(rst_b),
                  .pkt_in(inbound_pkts), .pkt_in_avail(inbound_pkts_avail),
                  .last_routed(last_routed), .req(channel_req),
                  .pkt_out(pkt_to_route), .pkt_out_avail(pkt_to_route_avail)); 

endmodule

/* **********************************************
 * I/O BUFFERS
 * **********************************************
 */

/*
 * wrapper holding input and output buffers to be connected
 * to channel_sel, routing, and node
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
 * Decides which packet from inputs to use
 * - constantly see which nodes have packets available
 * - request new packet when last one has been sent (1cycle)
 */
module channel_sel(clk, rst_b,
                   pkt_in, pkt_in_avail, last_routed,
                   pkt_out, pkt_out_avail, req);
  parameter ROUTERID = 0;
  input clk, rst_b;
  input pkt_t [3:0] pkt_in;
  input [3:0] pkt_in_avail;
  input last_routed;
  output pkt_t pkt_out;
  output pkt_out_avail;
  output [3:0] req;

  // round-robin-esque scheduling, starting from 0
  logic [3:0] use_p;   // one-hot port use
  reg [3:0][3:0] p_wait;
  reg [3:0] p_used;

  always_comb begin // the always_comb from heeeellllll............
    if (pkt_in_avail == 3'b0) use_p = 3'b0;
    if (pkt_in_avail[0] == 1'b1) 
      use_p[0] = (p_wait[0] >= p_wait[1] 
                  && p_wait[0] >= p_wait[2]
                  && p_wait [0] >= p_wait[3]) ? 1'b1 : 1'b0;
    if (pkt_in_avail[1] == 1'b1)
      use_p[1] = (p_wait[1] > p_wait[0]
                  && p_wait[1] >= p_wait[2]
                  && p_wait[1] >= p_wait[3]) ? 1'b1 : 1'b0;
    if (pkt_in_avail[2] == 1'b1)
      use_p[2] = (p_wait[2] > p_wait[0]
                  && p_wait[2] > p_wait[1]
                  && p_wait[2] >= p_wait[3]) ? 1'b1 : 1'b0;
    if (pkt_in_avail[3] == 1'b1)
      use_p[3] = (p_wait[3] > p_wait[0]
                  && p_wait[3] > p_wait[1]
                  && p_wait[3] > p_wait[2]) ? 1'b1 : 1'b0;
   end

  always_ff @(posedge clk, negedge rst_b) begin  
    // increment wait count unless invalid or last used
    if (~rst_b) p_wait = 0;
    else begin
      p_wait[0] = (p_used[0] || !pkt_in_avail[0]) ? 0 : p_wait[0] + 1;
      p_wait[1] = (p_used[1] || !pkt_in_avail[1]) ? 0 : p_wait[1] + 1;
      p_wait[2] = (p_used[2] || !pkt_in_avail[2]) ? 0 : p_wait[2] + 1;
      p_wait[3] = (p_used[3] || !pkt_in_avail[3]) ? 0 : p_wait[3] + 1;
    end
  end

  // hold last use_p val to reset p_wait at next cycle
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) p_used <= 0;
    else p_used <= use_p;
  end

  // store new packet when the last one has been sent
  reg [31:0] bfr;
  pkt_t channel;

  always_comb
    case(use_p)
      4'b0001: channel = pkt_in[0];
      4'b0010: channel = pkt_in[1];
      4'b0100: channel = pkt_in[2];
      4'b1000: channel = pkt_in[3];
      default: channel = 0;
   endcase 

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) bfr <= 0;
    else bfr <= (last_routed) ? channel : bfr;
  end

  // packet is available whenever buf has a value
  assign pkt_out_avail = (0),
         pkt_out = bfr;

  assign req = (last_routed) ? use_p : 0;  // TODO: this is wrong. fix it.

endmodule

/*
 * Gets a packet and pushes it and a valid bit to the an array of 4 pkts
 * - register holds pkt until recieved signal?
 */
module routing(clk, rst_b,
               pkt_in, pkt_in_avail, out_read,
               out_data, out_data_avail, last_routed);
  parameter ROUTERID = 0;
  input clk, rst_b, pkt_in_avail;
  input pkt_t pkt_in;
  input [3:0] out_read;                   // one-hot indication that packet went through
  output last_routed;
  output bit [3:0] out_data_avail;
  output pkt_t [3:0] out_data;

  logic [2:0] send_node;
  out_node #(ROUTERID) (.*);

  // indicate whether the packet last sent got through
  reg read;
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) read <= 0;
    else        read <= (out_read != 0) ? 1'b1 : 1'b0;
  end

  // assign outbound vals
  assign last_routed = read;

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
module out_node(clk, rst_b,
               pkt_in, pkt_in_avail,
               send_node);
  parameter ROUTERID = 0;
  input clk, rst_b, pkt_in_avail;
  input pkt_t pkt_in;
  output [2:0] send_node;

  logic [3:0] dest;
  assign dest = pkt_in.sourceID,
         send_node = (dest == ROUTERID) ? dest + 1 : 0; // TODO: fix this to work for both router ids!!

endmodule
