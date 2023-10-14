`timescale 1ns / 1ps
// =============================================================================
//  Program : memory_management.v
//  Author  : Chun Wei Chao
//  Date    : Nov/07/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is the dynamic memory manager interface to the Aquila processor core.
//  It is integrated direct to the processor microarchitecture signals so it
//  require tweaking if you want to integrate it to other processors.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
//
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2020,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

module memory_manager #(parameter XLEN = 32, parameter HEAP_SIZE = 32'h02000000
)
(
    input clk,
    input rst,
    input stall_i,
    input allocate_request,
    input [31:0] allocate_size,
    input free_request,
    input [31:0] free_addr,
    output [31:0] allocate_addr,
    output allocate_finish,
    output free_finish,
    output [31:0] mcounter,
    output [31:0] fcounter,

    input                      core_strobe_i,      // Processor send a request.
    input                      core_rw_i,          // 0 for read, 1 for write.
    input  [XLEN/8-1 : 0]      core_byte_enable_i, // Byte-enable signal.
    input  [XLEN-1 : 0]        core_addr_i,        // Memory addr for the request.
    input  [XLEN-1 : 0]        core_data_i,        // Data to main memory.
    output [XLEN-1 : 0]        core_data_o,        // Data from main memory.
    output                     core_ready_o,       // The cache data is ready.
    input                      core_flush_i,       // Cache flush signal.
    input                      core_is_amo_i,      // AMO request from core.
    input  [4 : 0]             core_amo_type_i,    // Type of AMO from core.

    output                     dmm_strobe,      
    output                     dmm_rw,          
    output  [XLEN/8-1 : 0]     dmm_byte_enable, 
    output  [XLEN-1 : 0]       dmm_addr,         
    output  [XLEN-1 : 0]       dmm_data_o,        
    input   [XLEN-1 : 0]       dmm_data_i,        
    input                      dmm_ready_i,       
    output                     dmm_flush_o,       
    output                     dmm_is_amo_o,      
    output  [4 : 0]            dmm_amo_type_o,    
    input                      busy_flushing_i
);

localparam Idle = 0,
           DMM = 1,
           CORE = 2;
reg  [1:0] S, S_nst;

// malloc ip communicate with free_buffer
wire                         state_idle;
wire                         state_analysis;
wire                         free_request_o;
wire [31:0]                  free_address_o;

// dmm communicate with cache
wire                         read_request;
wire [31:0]                  read_address, read_address_tmp;
wire                         write_request;
wire [31:0]                  write_address, write_address_tmp;
wire                         write_valid;
wire [31:0]                  write_data;
wire [3:0]                   write_len;
wire [3:0]                   read_len;
wire                         read_valid;
wire [31:0]                  read_data;
reg  [3:0]                   read_cnt;
reg  [3:0]                   write_cnt;
wire                         mm_strobe;
wire                         mm_rw;
wire [31:0]                   mm_address;
wire                         mm_finish;
reg                          core_strobe_r;
reg                          mm_strobe_r;

// module input & output
 
assign dmm_strobe = ((S == CORE) || (S == Idle && core_strobe_i)) ? core_strobe_r : (S == DMM) ? mm_strobe_r : 0;
assign dmm_rw = ((S == CORE) || (S == Idle && core_strobe_i)) ? core_rw_i : (S == DMM) ? mm_rw : 0;
assign dmm_byte_enable = ((S == CORE) || (S == Idle && core_strobe_i)) ? core_byte_enable_i : (S == DMM) ? 4'b1111 : 0;
assign dmm_addr = ((S == CORE) || (S == Idle && core_strobe_i)) ? core_addr_i : (S == DMM) ? mm_address : 0;
assign dmm_data_o = ((S == CORE) || (S == Idle && core_strobe_i)) ? core_data_i : (S == DMM) ? write_data : 0;
assign core_data_o = dmm_data_i;
assign core_ready_o = ((S == CORE) || (S == Idle && core_strobe_i)) ? dmm_ready_i : 0;
assign dmm_flush_o = (S == Idle) ? core_flush_i : 0;
assign dmm_is_amo_o = core_is_amo_i;
assign dmm_amo_type_o = core_amo_type_i;

/*
assign dmm_strobe = core_strobe_i;
assign dmm_rw = core_rw_i;
assign dmm_byte_enable = core_byte_enable_i;
assign dmm_addr = core_addr_i;
assign dmm_data_o = core_data_i;
assign core_data_o = dmm_data_i;
assign core_ready_o = dmm_ready_i;
assign dmm_flush_o = core_flush_i;
assign dmm_is_amo_o = core_is_amo_i;
assign dmm_amo_type_o = core_amo_type_i;
*/
// dmm signal
assign mm_strobe = write_request || read_request;
assign mm_rw = write_request ? 1 : 0;
assign mm_address = write_request ? write_address_tmp :
                    read_request ? read_address_tmp :
                    0;
assign mm_finish = write_request ? ((write_cnt == write_len-1) && dmm_ready_i) ||  (write_cnt == write_len):
                   read_request ? ((read_cnt == read_len-1) && dmm_ready_i)  ||  (read_cnt == read_len):
                   0;
assign write_valid = (S == DMM && dmm_ready_i);
assign read_valid = (S == DMM && dmm_ready_i);
assign read_data = dmm_data_i;
assign read_address_tmp = read_address + (read_cnt * 4);
assign write_address_tmp = write_address + (write_cnt * 4); 
// FSM
always @(posedge clk) begin
    S <= S_nst;
end

always @(*) begin
    case(S)
        Idle:
            if(core_strobe_r || core_strobe_i) S_nst = CORE;
            else if(mm_strobe_r) S_nst = DMM;
            else S_nst = Idle;
        CORE:
            if(dmm_ready_i) S_nst = Idle;
            else S_nst = CORE;
        DMM:
            if(mm_finish) S_nst = Idle;
            else S_nst = DMM;
        default:
            S_nst = Idle;
  endcase
end

always @(posedge clk) begin
    if((S == DMM || S == Idle) && core_strobe_i) core_strobe_r <= 1;
    else if(S == CORE) core_strobe_r <= 0;
    else core_strobe_r <= core_strobe_r;
end

always @(posedge clk) begin
    if(S == Idle) mm_strobe_r <= mm_strobe;
    else if(S == DMM && !mm_finish && dmm_ready_i) mm_strobe_r <= 1;
    else mm_strobe_r <= 0;
end

always @(posedge clk) begin
    if(S == Idle) write_cnt <= 0;
    else if (S == DMM && dmm_ready_i) write_cnt <= write_cnt + 1;
    else  write_cnt <= write_cnt;
end

always @(posedge clk) begin
    if(S == Idle) read_cnt <= 0;
    else if (S == DMM && dmm_ready_i) read_cnt <= read_cnt + 1;
    else  read_cnt <= read_cnt;
end

newlib_based_allocator #(.HEAP_SIZE(HEAP_SIZE)) allocator(
   .clk(clk), 
   .rst(rst), 
   .stall_i(stall_i),
   /* API IO port */
   .allocate_request(allocate_request),
   .free_request(free_request_o), 
   .free_addr(free_address_o), 
   .allocate_size(allocate_size),
   .heap_start_address(32'h90110004),
   .allocate_addr(allocate_addr), 
   .allocate_finish(allocate_finish),
   
   /* master IO port */
   .read_valid(read_valid),
   .read_data(read_data), 
   .write_valid(write_valid),  
   .read_request(read_request), 
   .write_request(write_request), 
   .read_address(read_address),  
   .write_address(write_address), 
   .write_data(write_data),
   .write_len(write_len),
   .read_len(read_len),
   
   /* others */  
   .state_idle(state_idle), 
   .state_analysis(state_analysis),
   .mcounter(mcounter),
   .fcounter(fcounter)
);
     
free_circular_buffer c1(
  .clk(clk), 
  .rst(rst), 
  .free_request_i(free_request), 
  .free_address_i(free_addr), 
  .idle(state_idle), 
  .analysis(state_analysis), 
  .free_finish(free_finish), 
  .free_request_o(free_request_o), 
  .free_address_o(free_address_o)
);       

endmodule
