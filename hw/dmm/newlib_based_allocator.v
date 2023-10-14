`timescale 1ns / 1ps
// =============================================================================
//  Program : newlib_based_allocator.v
//  Author  : Sheng Di Hong
//  Date    : May/10/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the dynamic memory manager based on the Newlib malloc() algorithm.
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
//  Copyright 2018,
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

module newlib_based_allocator
#(parameter 
        HEAP_SIZE          = 32'h02000000 //32MB.  
  )
  (
  input clk,
  input rst,
  input stall_i,
  input                             allocate_request,
  input                             free_request,
  input [31:0]                      free_addr,
  input [31:0]                      allocate_size,
  input [31:0]                      heap_start_address,
  input                             read_valid,
  input [31:0]                      read_data,
  input                             write_valid,
                                                              
  output reg [31:0]                 allocate_addr,
  output reg [2:0]                  allocate_finish,
  output reg                        read_request,
  output reg                        write_request,
  output reg [31:0]                 write_address,
  output reg [31:0]                 read_address,
  output reg [31:0]                 write_data,
  output                            state_idle,
  output reg [3 :0]                 write_len,
  output reg [3 :0]                 read_len,
  output                            state_analysis,
  output [31:0]                      mcounter,
  output [31:0]                      fcounter
);

reg [31:0] alloc_counter;
reg [31:0] free_counter;
reg [3:0] bstate;

assign mcounter = alloc_counter;
assign fcounter = free_counter;
/* main state signal */
reg allocate_request_r;
reg [1:0]idle_cnt;
reg [2:0] share_cnt;
reg [3:0]MAIN_ST;
reg [3:0]MAIN_NST;
reg [6:0] bin_at;
reg [6:0] bin_at_r;
reg [6:0] exact_bin_at;
reg [6:0] others_bin_at;
reg [7:0] search_bin_idx;
reg [7:0] search_bin_idx_r;
reg [31:0] allocate_size_align;
reg [31:0] allocate_size_r;
reg [31:0] top_size;
reg [0:127] bin_bitmap;
reg [0:127] bin_bitmap_tmp;
reg [0:127] bin_mask;
wire exact_bin_flag;
wire other_bin_flag;
wire top_bin_flag;
wire[31:0] allocate_size_tmp;


/* main FSM parameter */
localparam INIT     = 0 , /* initial top size */
           IDLE     = 1 , /* idle state */
		   BUFF     = 2 , /* wait 1 cycle to read bin_at_r */
		   ANALYSIS = 3 , /* analysis the allocate space was found from which mode */
		   EXACT    = 4 , /* drive to exact FSM */
		   OTHERS   = 5 , /* drive to others FSM */
		   TOP      = 6 , /* drive to top FSM */
		   INSERT   = 7 , /* drive to insert FSM */
		   FREE     = 8 , /* drive to free FSM */
		   FAIL     = 9 , /* allocate fail */
		   REALLOC  = 10, /* drive to exact realloc */
		   DONE     = 11  /* done state */
		   ;
		   


/* memory control signal */
reg [2:0] MEM_CTRL_ST;
reg [2:0] MEM_CTRL_NST;
reg [2:0] read_len_cnt;
reg [2:0] write_len_cnt;
wire start_read;
wire start_write;
wire write_complete;
wire read_complete;


/* memory control state machine parameter */
localparam 
  MEM_IDLE       = 0, /* idle state */
  MEM_READ_REQ   = 1, /* send read request */
  MEM_WAIT_READ  = 2, /* wait to read data */
  MEM_WRITE_REQ  = 3, /* send to write data */
  MEM_WAIT_WRITE = 4, /* wait to write data */
  MEM_DONE       = 5  /* done state */
  ;
		   

/* top control signal */
reg  [ 2:0] TOP_ST;
reg  [ 2:0] TOP_NST;
wire [31:0] top_allocate_address;
wire [31:0] top_write_addr;
wire [31:0] top_write_data;
wire top_write_len;
wire top_write_flag;
wire [31:0]top_addr;


/* top state machine parameter */
localparam 
  TOP_IDLE     = 0, /* idle state */
  TOP_OUTPUT   = 1, /* output the allocate address */
  TOP_SET_SIZE = 2, /* set allocate size in head */
  TOP_UPDATE   = 3, /* updare the top size */
  TOP_DONE     = 4  /* done state */
  ;


/* exact control signal */
reg  [ 2:0] EXACT_ST;
reg  [ 2:0] EXACT_NST;
reg  [31:0] alloc_chunk_info [0:3];
reg  [31:0] read_chunk_info;
wire last_fd;
wire last_bk;
wire exact_rw_flag;
wire exact_write_flag;
wire exact_read_flag;
wire exact_write_len;
wire [2:0]exact_read_len;
wire [31:0] exact_allocate_address;
wire [31:0] exact_write_addr;
wire [31:0] exact_write_data;
wire [31:0] exact_read_addr;
wire [31:0] exact_read_data;


/* exact state machine parameter */
localparam 
  EXACT_IDLE            = 0, /* state idle */
  EXACT_OUTPUT          = 1, /* output allocate address and read the bin cache*/
  EXACT_UNLINK_FD       = 2, /* unlink the allocate chunk from forward chunk in free list */
  EXACT_UNLINK_BK       = 3, /* unlink the allocate chunk from back chunk in free list */
  EXACT_SET_NEXTCHUNK   = 4, /* set next chunk -> previous in use bit to 1 */
  EXACT_UPDATE_EMPTY    = 5, /* update the bin cache */
  EXACT_UPDATE_NEW      = 6, /* update the bin cache */
  EXACT_DONE            = 7  /* done state */
  ;
  
/* others control signal */
reg  [ 2:0] OTHERS_ST;
reg  [ 2:0] OTHERS_NST;
wire [ 2:0] others_read_len;
wire [31:0] others_allocate_address;
wire [31:0] others_write_addr;
wire [31:0] others_write_data;
wire [31:0] others_read_addr;
wire others_write_len;
wire others_write_flag;
wire others_read_flag;


/* others state machine parameter */
localparam 
  OTHERS_IDLE            = 0, /* idle state */
  OTHERS_OUTPUT          = 1, /* output allocate address and read the bin cache */
  OTHERS_SET_SIZE        = 2, /* set up the allocate size in the head */
  OTHERS_UNLINK_FD       = 3, /* unlink the allocate chunk from forward chunk in free list */
  OTHERS_UNLINK_BK       = 4, /* unlink the allocate chunk from back chunk in free list */
  OTHERS_UPDATE_EMPTY    = 5, /* update bin cache */
  OTHERS_UPDATE_NEW      = 6, /* update bin cache */
  OTHERS_DONE            = 7
  ;

  
/* insert control signal */
reg insert_update_cache;
reg [3:0] INSERT_ST;
reg [3:0] INSERT_NST;
reg [31:0]insert_address;
reg [31:0]insert_size;
reg [31:0]current_pointer;
reg [31:0]insert_chunk_info[0:3];
wire [31:0]insert_write_addr;
wire [31:0]insert_write_data;
wire [31:0]insert_read_addr;
wire insert_small_size;
wire search_success;
wire insert_not_empty_bin;
wire insert_smallest;
wire insert_write_flag;
wire insert_read_flag;
wire [1:0] insert_write_len;
wire [2:0] insert_read_len;


/* insert state parameter */
localparam 
  INSERT_IDLE           = 0 , /* idle state */
  INSERT_DELAY          = 1 , /* delay 1 cycle to read bin_at_r data */
  INSERT_READ_BIN       = 2 , /* read the first chunk information from the bin cache  */
  INSERT_ANALYSIS       = 3 , /* analysis the insert circumstances */
  INSERT_READ_CHUNK     = 4 , /* read free bin list chunk */
  INSERT_LINK_FD        = 5 , /* link forward chunk in free list */
  INSERT_LINK_BK        = 6 , /* link back chunk in free list */
  INSERT_SET_HEAD       = 7 , /* set up the chunk information */
  INSERT_SET_FOOT       = 8 , /* set up the insert size in next chunk->previous_size*/
  INSERT_UPDATE         = 9 , /* update bin cache if need */
  INSERT_DONE           = 10  /* done state */
  ;
  

  
/* free control signal */
reg update_fd_cache;
reg previous_inuse;
reg top_check_complete;
reg [3:0] FREE_ST;
reg [3:0] FREE_NST;
reg [31:0]free_size_r;
reg [31:0]free_address_r;
wire in_top;
wire next_in_top;
wire free_write_flag;
wire free_read_flag;
wire [31:0]free_write_addr;
wire [31:0]free_write_data;
wire [31:0]free_read_addr;
wire [31:0]free_read_data;
wire [31:0]previous_address;
wire [2:0]free_read_len;
wire [2:0]free_write_len;		   



/* free state parameter */
localparam 
  FREE_IDLE            = 0 , /* idle state */
  FREE_READ_SIZE       = 1 , /* according to the address to read free size */
  FREE_INITIAL         = 2 , /* clean the free data */
  FREE_ANALYSIS        = 3 , /* analysis previous node is free or not */
  FREE_READ_PREVIOUS   = 4 , /* read previous chunk information */
  FREE_UNLINK_FD       = 5 , /* unlink the forward chunkd from free list structure */
  FREE_UNLINK_BK       = 6 , /* unlink the back chunkd from free list structure */
  FREE_UPDATE_EMPTY    = 7 , /* update the bin cache */
  FREE_UPDATE_NEW      = 8 , /* update the bin cache */
  FREE_CHECK_TOP       = 9 , /* check this free chunk whether need to merge with top */
  FREE_UPDATE_TOP      = 10, /* update top size */
  FREE_READ_NEXT       = 11, /* read next chunk information */
  FREE_CLEAR_USEBIT    = 12, /* clear usebit from next_size[0] */
  FREE_READ_USEBIT     = 13, /* read next usebit from next_chunk->next_size[0] */
  FREE_TOP_DONE        = 14, /* free done */
  FREE_NTOP_DONE       = 15  /* free done and need to insert */
  ;

integer i;
//bin cache port A 
reg [8:0] bin_addr_w;
reg [8:0] bin_addr_r;
reg [31:0] bin_data_w;
reg [31:0] bin_data_r;
wire bin_we;
wire [9:0] bin_addr;
wire [31:0] bin_data_in;
wire [31:0] bin_data_out;

//bin cache port B
reg [8:0] chunk_ptr_addr_w;
reg [8:0] chunk_ptr_addr_r;
reg [31:0] chunk_ptr_w;
reg [31:0] chunk_ptr_r;
wire  chunk_ptr_we;
wire [9:0] chunk_ptr_addr;
wire [31:0] chunk_ptr_in;
wire [31:0] chunk_ptr_out;
wire [31:0]other_bin_size;





/* output to the free circular buffer controller to update data */
assign state_idle     = MAIN_ST == IDLE;
assign state_analysis = (MAIN_ST == FREE) & (FREE_ST == FREE_NTOP_DONE | FREE_ST == FREE_TOP_DONE);





always @(posedge clk)begin
  if(TOP_ST == TOP_OUTPUT)begin
    allocate_addr <= top_allocate_address;
	allocate_finish <= 1;
  end
  else if(EXACT_ST == EXACT_OUTPUT & share_cnt == 0)begin
    allocate_addr <= exact_allocate_address;
	allocate_finish <= 1;
  end
  else if(OTHERS_ST == OTHERS_OUTPUT & share_cnt == 0)begin
    allocate_addr <= others_allocate_address;
	allocate_finish <= 1;
  end
  else if(MAIN_ST == FAIL)begin
    allocate_addr <= 0;
	allocate_finish <= 4;
  end
  //when allocate finish but cpu is stall for other cause
  //need to save result of allocate
  else if(stall_i) begin
    allocate_addr <= allocate_addr;
    allocate_finish <= allocate_finish;
  end
  else begin
    allocate_addr <= allocate_addr;
    allocate_finish <= 0;
  end
end

assign exact_bin_flag = /*(MAIN_ST == ANALYSIS) &*/ 
                        ((allocate_size_align<504 & (bin_bitmap[bin_at_r] | bin_bitmap[bin_at_r+1])) | // if belong to small bin,also scan the next one
						(bin_data_out >= allocate_size_r & bin_data_out < allocate_size_r+16 & bin_bitmap[bin_at_r] ) |
						(other_bin_size >= allocate_size_r & other_bin_size < allocate_size_r+16 & bin_bitmap[search_bin_idx_r] )
						);    // else check allocate size isn't need to split
assign other_bin_flag = (MAIN_ST == ANALYSIS) & ((!search_bin_idx_r[7]) & (search_bin_idx_r >= 2 & other_bin_size >= allocate_size_r+16) | (bin_data_out >= allocate_size_r+16)) ;//For each possibly nonempty bin
assign top_bin_flag   = (MAIN_ST == ANALYSIS) & top_size >= allocate_size_r;//Try to use top chunk
assign other_bin_size = chunk_ptr_out;

always @(*)begin
  if(allocate_size_align<504)
    exact_bin_at = bin_bitmap[bin_at_r] ? {bin_at_r} :{bin_at_r+7'd1};
  else if(bin_data_out >= allocate_size_r & bin_data_out < allocate_size_r+16 & bin_bitmap[bin_at_r])
    exact_bin_at = {bin_at_r};
  else 
    exact_bin_at = search_bin_idx_r[6:0];
end


always @(*)begin
  if(bin_data_out >= allocate_size_r+16)
    others_bin_at = bin_at_r;
  else 
    others_bin_at = search_bin_idx_r[6:0];
end

always @(posedge clk)begin
  if(MAIN_ST == INIT)
    idle_cnt <= 0;
  else if(MAIN_ST == IDLE)
    idle_cnt <= (idle_cnt == 2) ? 2'd2 : idle_cnt + 2'd1;
  else
    idle_cnt <= 0;
end

always @(posedge clk)begin
  if(MAIN_ST == INIT) begin
    free_counter <= 0;
    alloc_counter <= 0;
  end
  else begin
    if(MAIN_ST == TOP || MAIN_ST == EXACT || MAIN_ST == OTHERS || (bstate == OTHERS && MAIN_ST == INSERT))
      alloc_counter <= alloc_counter + 1;
    else
      alloc_counter <= alloc_counter;
    if(MAIN_ST == FREE || (bstate == FREE && MAIN_ST == INSERT))
      free_counter <= free_counter + 1;
    else
      free_counter <= free_counter;
  end
end

always @(posedge clk) begin
  if(MAIN_ST != MAIN_NST)
    bstate <= MAIN_ST;
  else
    bstate <= bstate;
end
always @(posedge clk)begin
  if(rst)
    MAIN_ST <= INIT;
  else 
    MAIN_ST <= MAIN_NST;
end

always @(*)begin
  case(MAIN_ST)
  INIT:
	  MAIN_NST = IDLE;
	IDLE:
	  if(allocate_request_r & idle_cnt == 2) MAIN_NST = BUFF;
	  else if(free_request)MAIN_NST = FREE;
	  else MAIN_NST = IDLE;
	BUFF:
	  MAIN_NST = ANALYSIS;
	ANALYSIS:
	  if(exact_bin_flag) MAIN_NST = EXACT;
	  else if(other_bin_flag) MAIN_NST = OTHERS;
	  else if(top_bin_flag)  MAIN_NST = TOP;
	  else MAIN_NST = FAIL;
	TOP:
	  if(TOP_ST == TOP_DONE) MAIN_NST = DONE;
	  else MAIN_NST = TOP;
	EXACT:
	  if(EXACT_ST == EXACT_DONE) MAIN_NST = DONE;
	  else MAIN_NST = EXACT;
	OTHERS:
	  if(OTHERS_ST == OTHERS_DONE) MAIN_NST = INSERT;
	  else MAIN_NST = OTHERS;
	INSERT:
	  if(INSERT_ST == INSERT_DONE) MAIN_NST = DONE;
	  else MAIN_NST = INSERT;
	FREE:
	  if(FREE_ST == FREE_TOP_DONE) MAIN_NST = DONE;
	  else if(FREE_ST == FREE_NTOP_DONE) MAIN_NST = INSERT;
	  else MAIN_NST = FREE;
	FAIL:
	  MAIN_NST = DONE;
	DONE:
	  MAIN_NST = IDLE;
	default:
	  MAIN_NST = INIT;
  endcase
end




// ------------------------------------
// chunk information controller
// ------------------------------------
assign bin_we = (EXACT_ST == EXACT_UPDATE_EMPTY) || (EXACT_ST == EXACT_UPDATE_NEW & read_valid) ||
				(OTHERS_ST == OTHERS_UPDATE_EMPTY) || (OTHERS_ST == OTHERS_UPDATE_NEW & read_valid ) ||
				(INSERT_ST == INSERT_UPDATE & share_cnt > 0) || (INSERT_ST == INSERT_LINK_BK & insert_update_cache) ||
				(FREE_ST == FREE_UPDATE_EMPTY) || (FREE_ST == FREE_UPDATE_NEW & read_valid ) || (FREE_ST == FREE_UNLINK_BK & update_fd_cache )
				;
assign bin_addr = {1'b0,bin_addr_w};
assign bin_data_in = bin_data_w;


always @(*)begin
  if(MAIN_ST == IDLE)
    bin_addr_w = {bin_at_r,2'd0}+9'd1;
  else if(MAIN_ST == BUFF)
    bin_addr_w = {bin_at_r,2'd0}+9'd1;
  else if(MAIN_ST == ANALYSIS)begin
    if(exact_bin_flag)
	  bin_addr_w = {exact_bin_at,2'd0}+9'd1;  
	else if(other_bin_flag)
	  bin_addr_w = {others_bin_at,2'd0}+9'd1;
	else //if(top_bin_flag)
      bin_addr_w = 0;
  end
  else if(FREE_ST == FREE_UNLINK_BK) /* update fd bin cache if need */
    bin_addr_w = {bin_at_r,2'b11};    
  else if(INSERT_ST == INSERT_LINK_BK) /* update fd bin cache if need */
    bin_addr_w = {bin_at_r,2'b11};    
  else 
    bin_addr_w = {bin_at_r,share_cnt[1:0]};
end


always @(*)begin
  if(EXACT_ST == EXACT_UPDATE_EMPTY)
    bin_data_w = 0;
  else if(EXACT_ST == EXACT_UPDATE_NEW & read_valid)
    bin_data_w = read_data;
  else if(OTHERS_ST == OTHERS_UPDATE_EMPTY)
    bin_data_w = 0;
  else if(FREE_ST == FREE_UPDATE_EMPTY)
    bin_data_w = 0;
  else if(OTHERS_ST == OTHERS_UPDATE_NEW & read_valid)
    bin_data_w = read_data;
  else if(FREE_ST == FREE_UPDATE_NEW & read_valid)
    bin_data_w = read_data;
  else if(FREE_ST == FREE_UNLINK_BK)
    bin_data_w = alloc_chunk_info[3];
  else if(INSERT_ST == INSERT_LINK_BK)
    bin_data_w = insert_address;
  else if(INSERT_ST == INSERT_UPDATE)
    bin_data_w = insert_chunk_info[share_cnt[1:0]];
  else 
    bin_data_w = bin_data_r;
end


always @(posedge clk)begin
  bin_data_r <= bin_data_w;
end



// ------------------------------------------------------------------------
//                      chunk pointer controller
// ------------------------------------------------------------------------
assign chunk_ptr_we = 
					  (EXACT_ST == EXACT_UPDATE_EMPTY) | (EXACT_ST == EXACT_UPDATE_NEW & read_valid) |
					  (OTHERS_ST == OTHERS_UPDATE_EMPTY) | (OTHERS_ST == OTHERS_UPDATE_NEW & read_valid) |
					  (FREE_ST == FREE_UPDATE_EMPTY) | (FREE_ST == FREE_UPDATE_NEW & read_valid) | 
					  (INSERT_ST == INSERT_UPDATE)
					  ;
assign chunk_ptr_in = chunk_ptr_w;
assign chunk_ptr_addr = (MAIN_ST == BUFF) ? {1'b0,chunk_ptr_addr_w} : {1'b1,chunk_ptr_addr_w};

always @(*)begin
  if(MAIN_ST == ANALYSIS)begin
    if(exact_bin_flag)
	  chunk_ptr_addr_w = exact_bin_at;//free chunk size gap is less than 16 byte    
	else if(other_bin_flag)
	  chunk_ptr_addr_w = others_bin_at;
	else //if(top_bin_flag)
      chunk_ptr_addr_w = 0;
  end
  else if(MAIN_ST == BUFF)
    chunk_ptr_addr_w = {search_bin_idx,2'd0}+9'd1; //correspond bin address
  else if(EXACT_ST == EXACT_UPDATE_EMPTY)
    chunk_ptr_addr_w = bin_at_r;
  else if(EXACT_ST == EXACT_UPDATE_NEW)
    chunk_ptr_addr_w = bin_at_r;
  else if(OTHERS_ST == OTHERS_UPDATE_EMPTY)
    chunk_ptr_addr_w = bin_at_r;
  else if(OTHERS_ST == OTHERS_UPDATE_NEW & read_valid)
    chunk_ptr_addr_w = bin_at_r;
  else if(FREE_ST == FREE_UPDATE_NEW & read_valid)
    chunk_ptr_addr_w = bin_at_r;
  else if(FREE_ST == FREE_READ_NEXT)
    chunk_ptr_addr_w = bin_at_r;
  else if(FREE_ST == FREE_READ_PREVIOUS)
    chunk_ptr_addr_w = bin_at_r;
  else if(INSERT_ST == INSERT_READ_BIN)
    chunk_ptr_addr_w = bin_at_r;
  else if(FREE_ST == FREE_UPDATE_EMPTY)
    chunk_ptr_addr_w = bin_at_r;
  else if(INSERT_ST == INSERT_UPDATE)
    chunk_ptr_addr_w = bin_at_r;
  else 
    chunk_ptr_addr_w = chunk_ptr_addr_r;    
end


always @(posedge clk)begin
  chunk_ptr_addr_r <= chunk_ptr_addr_w;
end


always @(*)begin
  if(EXACT_ST == EXACT_UPDATE_EMPTY)
    chunk_ptr_w = 0;
  else if(EXACT_ST == EXACT_UPDATE_NEW)
    chunk_ptr_w = alloc_chunk_info[3];
  else if(OTHERS_ST == OTHERS_UPDATE_EMPTY)
    chunk_ptr_w = 0;
  else if(FREE_ST == FREE_UPDATE_EMPTY)
    chunk_ptr_w = 0;
  else if(OTHERS_ST == OTHERS_UPDATE_NEW)
    chunk_ptr_w = alloc_chunk_info[3];
  else if(FREE_ST == FREE_UPDATE_NEW)
    chunk_ptr_w = alloc_chunk_info[3];
  else if(INSERT_ST == INSERT_UPDATE)
    chunk_ptr_w = insert_address;
  else
    chunk_ptr_w = chunk_ptr_r;
end


always @(posedge clk)begin
  chunk_ptr_r <= chunk_ptr_w;
end







// ------------------------------------
//store allocate request
// ------------------------------------
always @(posedge clk)begin
  if(MAIN_ST == IDLE)
    allocate_request_r <= (allocate_request) ? allocate_request : 0;
  else if(MAIN_ST == ANALYSIS)
    allocate_request_r <= 0;
  else 
    allocate_request_r <= (allocate_request) ? allocate_request : allocate_request_r;
end

// ------------------------------------
//store allocate size
// ------------------------------------
always @(posedge clk)begin
  if(MAIN_ST == IDLE)
	allocate_size_r <= allocate_size_align;
  else 
    allocate_size_r <= allocate_size_r;
end

// ------------------------------------
//store correspond bin location 
// ------------------------------------
always @(posedge clk)begin
  if(MAIN_ST == IDLE)
	bin_at_r <= bin_at;
  else if(MAIN_ST == ANALYSIS)begin
    if(exact_bin_flag)
	  bin_at_r <= exact_bin_at;//free chunk size gap is less than 16 byte
	else if(other_bin_flag)
	  bin_at_r <= others_bin_at;
  end
  else if(MAIN_ST == INSERT || MAIN_ST == FREE)
    bin_at_r <= bin_at;
end


// ------------------------------------
//store other bin location 
// ------------------------------------
always @(posedge clk)begin
  if(MAIN_ST == BUFF)
	search_bin_idx_r <= search_bin_idx;
  else 
    search_bin_idx_r <= search_bin_idx_r;
end


// ------------------------------------------------------------------------
//                          memory controller begin
// ------------------------------------------------------------------------

assign start_write = top_write_flag | exact_write_flag | others_write_flag | free_write_flag | insert_write_flag ;
assign write_complete = write_valid & write_len_cnt==1; //once burst will complete
assign start_read = exact_read_flag | others_read_flag | free_read_flag | insert_read_flag;
assign read_complete = read_valid & read_len_cnt==1; 

always @(posedge clk)begin
  if(rst)
    MEM_CTRL_ST <= MEM_IDLE;
  else
    MEM_CTRL_ST <= MEM_CTRL_NST;
end

always @(*)begin
  case(MEM_CTRL_ST)
     MEM_IDLE:
	   if(start_read)
	     MEM_CTRL_NST = MEM_READ_REQ;
	   else if(start_write) MEM_CTRL_NST = MEM_WRITE_REQ;
	   else MEM_CTRL_NST = MEM_IDLE;
	 MEM_READ_REQ: 
	   MEM_CTRL_NST = MEM_WAIT_READ;
	 MEM_WAIT_READ:	 
	   if(read_complete) MEM_CTRL_NST = (exact_rw_flag) ? MEM_WRITE_REQ : MEM_DONE;
	   else MEM_CTRL_NST = MEM_WAIT_READ;
	 MEM_WRITE_REQ: 
	   MEM_CTRL_NST = MEM_WAIT_WRITE;
	 MEM_WAIT_WRITE:
	   if(write_complete) MEM_CTRL_NST = MEM_DONE;
	   else MEM_CTRL_NST = MEM_WAIT_WRITE;
     MEM_DONE:    
	   MEM_CTRL_NST = MEM_IDLE;
     default :
	   MEM_CTRL_NST = MEM_IDLE;
  endcase
end



// ------------------------------------
//read memory signal
// ------------------------------------
always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    read_request <= 0;
  else if(MEM_CTRL_ST == MEM_READ_REQ | ((MEM_CTRL_ST == MEM_WAIT_READ) && !read_complete))
    read_request <= 1;
  else
    read_request <= 0;
end

always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    read_address <= 0;
  else if(MEM_CTRL_ST == MEM_READ_REQ)begin
    if(MAIN_ST == EXACT)
      read_address <= exact_read_addr;
    else if(MAIN_ST == OTHERS)
	  read_address <= others_read_addr;
    else if(MAIN_ST == INSERT)
      read_address <= insert_read_addr;
    else if(MAIN_ST == FREE)
      read_address <= free_read_addr;
  end
end

always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    read_len <= 0;
  else if(MEM_CTRL_ST == MEM_READ_REQ)begin	
    if(MAIN_ST == EXACT)
	  read_len <= exact_read_len;
    else if(MAIN_ST == OTHERS)
	  read_len <= others_read_len;
    else if(MAIN_ST == FREE)
	  read_len <= free_read_len;
    else if(MAIN_ST == INSERT)
	  read_len <= insert_read_len;
  end
end

always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    read_len_cnt <= 0;
  else if(MEM_CTRL_ST == MEM_READ_REQ)begin	
    if(MAIN_ST == EXACT)
	  read_len_cnt <= exact_read_len;
    else if(MAIN_ST == OTHERS)
	  read_len_cnt <= others_read_len;
    else if(MAIN_ST == FREE)
	  read_len_cnt <= free_read_len;
    else if(MAIN_ST == INSERT)
	  read_len_cnt <= insert_read_len;
  end
  else if(MEM_CTRL_ST == MEM_WAIT_READ & read_valid)
    read_len_cnt <= read_len_cnt - 3'd1;
end



always @(posedge clk)begin
  if(EXACT_ST == EXACT_OUTPUT)
    share_cnt <= share_cnt + 3'd1;
  else if(EXACT_ST == EXACT_UPDATE_EMPTY)
    share_cnt <= share_cnt + 3'd1;
  else if(EXACT_ST == EXACT_UPDATE_NEW & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(OTHERS_ST == OTHERS_OUTPUT)
    share_cnt <= share_cnt + 3'd1;
  else if(OTHERS_ST == OTHERS_UPDATE_EMPTY)
    share_cnt <= share_cnt + 3'd1;
  else if(OTHERS_ST == OTHERS_UPDATE_NEW & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(INSERT_ST == INSERT_READ_BIN)
    share_cnt <= share_cnt + 3'd1;
  else if(INSERT_ST == INSERT_SET_HEAD & write_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(INSERT_ST == INSERT_READ_CHUNK & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(INSERT_ST == INSERT_UPDATE)
    share_cnt <= share_cnt + 3'd1;
  else if(FREE_ST == FREE_UPDATE_EMPTY)
    share_cnt <= share_cnt + 3'd1;
  else if(FREE_ST == FREE_READ_PREVIOUS & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(FREE_ST == FREE_READ_SIZE & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(FREE_ST == FREE_READ_NEXT & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(FREE_ST == FREE_UPDATE_NEW & read_valid)
    share_cnt <= share_cnt + 3'd1;
  else if(MEM_CTRL_ST == MEM_WAIT_READ)
    share_cnt <= share_cnt;
  else if(MEM_CTRL_ST == MEM_WAIT_WRITE)
    share_cnt <= share_cnt;
  else 
    share_cnt <= 0;
end

//store chunk information from bin cache or read data
always @(posedge clk)begin
  if(EXACT_ST == EXACT_OUTPUT)
    alloc_chunk_info[share_cnt-1] <= bin_data_out;
  else if(OTHERS_ST == OTHERS_OUTPUT)
    alloc_chunk_info[share_cnt-1] <= bin_data_out;
  else if(INSERT_ST == INSERT_READ_BIN)
    alloc_chunk_info[share_cnt-1] <= bin_data_out;
  else if(INSERT_ST == INSERT_READ_CHUNK & read_valid)
    alloc_chunk_info[share_cnt] <= read_data;
  else if(FREE_ST == FREE_READ_PREVIOUS & read_valid)
    alloc_chunk_info[share_cnt] <= read_data;
  else if(FREE_ST == FREE_READ_SIZE & read_valid)
    alloc_chunk_info[share_cnt] <= read_data;
  else if(FREE_ST == FREE_READ_NEXT & read_valid)
    alloc_chunk_info[share_cnt] <= read_data;
end

// ------------------------------------
//write memory signal
// ------------------------------------
always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    write_request <= 0;
  else if(MEM_CTRL_ST == MEM_WRITE_REQ | ((MEM_CTRL_ST == MEM_WAIT_WRITE) && !write_complete))
    write_request <= 1;
  else
    write_request <= 0;
end
	
always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    write_address <= 0;
  else if(MEM_CTRL_ST == MEM_WRITE_REQ)begin
    if(MAIN_ST == TOP) 
      write_address <= top_write_addr;
	else if(MAIN_ST == EXACT)
	  write_address <= exact_write_addr;
	else if(MAIN_ST == OTHERS)
	  write_address <= others_write_addr;
	else if(MAIN_ST == INSERT)
	  write_address <= insert_write_addr;
	else if(MAIN_ST == FREE)
	  write_address <= free_write_addr;
  end
end

always @(*)begin
  write_data = 0;
  if(MEM_CTRL_ST == MEM_WAIT_WRITE)begin
    if(MAIN_ST == TOP) 
      write_data = top_write_data;
	else if(MAIN_ST == EXACT)
	  write_data = exact_write_data;
	else if(MAIN_ST == OTHERS)
	  write_data = others_write_data;
	else if(MAIN_ST == INSERT)
	  write_data = insert_write_data;
	else if(MAIN_ST == FREE)
	  write_data = free_write_data;
  end
end



always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    write_len <= 0;
  else if(MEM_CTRL_ST == MEM_WRITE_REQ)begin	
    if(MAIN_ST == TOP)
	  write_len <= top_write_len;	
    else if(MAIN_ST == EXACT)
	  write_len <= exact_write_len;
    else if(MAIN_ST == OTHERS)
	  write_len <= others_write_len;
    else if(MAIN_ST == FREE)
	  write_len <= free_write_len;
    else if(MAIN_ST == INSERT)
	  write_len <= insert_write_len;
  end
end


/* write len controller */
always @(posedge clk)begin
  if(MEM_CTRL_ST == MEM_IDLE)
    write_len_cnt <= 0;
  else if(MEM_CTRL_ST == MEM_WRITE_REQ)begin	
    if(MAIN_ST == TOP)
	  write_len_cnt <= top_write_len;	
    else if(MAIN_ST == EXACT)
	  write_len_cnt <= exact_write_len;
    else if(MAIN_ST == OTHERS)
	  write_len_cnt <= others_write_len;
    else if(MAIN_ST == FREE)
	  write_len_cnt <= free_write_len;
    else if(MAIN_ST == INSERT)
	  write_len_cnt <= insert_write_len;
  end
  else if(MEM_CTRL_ST == MEM_WAIT_WRITE & write_valid)
    write_len_cnt <= write_len_cnt - 3'd1;
end

// ------------------------------------------------------------------------
//                     memory controller end
// ------------------------------------------------------------------------





// ------------------------------------------------------------------------
//                         allocate from top bin  
//                         and below is controller
// ------------------------------------------------------------------------


assign top_allocate_address = top_addr+4;
assign top_write_addr = top_addr;
assign top_write_data = (allocate_size_r | 32'h1); //fill previous in use bit in current size block
assign top_write_flag = TOP_ST == TOP_SET_SIZE;
assign top_write_len = 1;
assign top_addr = heap_start_address + (HEAP_SIZE-top_size);


always @(posedge clk)begin
  if(MAIN_ST == INIT)
    top_size <= HEAP_SIZE;
  else if(TOP_ST == TOP_UPDATE)
    top_size <= top_size - allocate_size_r; 
  else if(FREE_ST == FREE_UPDATE_TOP)
    top_size <= top_size + {free_size_r[31:1],1'b0};
end 


always @(posedge clk)begin
  if(rst)
    TOP_ST <= TOP_IDLE;
  else
    TOP_ST <= TOP_NST;
end


always @(*)begin
  case(TOP_ST)
     TOP_IDLE:    
	   if(MAIN_ST == TOP) TOP_NST = TOP_OUTPUT;
	   else TOP_NST = TOP_IDLE;
	 TOP_OUTPUT:
	   TOP_NST = TOP_SET_SIZE;
	 TOP_SET_SIZE:
	   if(MEM_CTRL_ST == MEM_DONE) TOP_NST = TOP_UPDATE;
	   else TOP_NST = TOP_SET_SIZE;
	 TOP_UPDATE:
	   TOP_NST = TOP_DONE;
	 TOP_DONE:
	   TOP_NST = TOP_IDLE;
	 default: 
	   TOP_NST = TOP_IDLE;
  endcase
end

// ------------------------------------------------------------------------
//                      allocate from top bin end
// ------------------------------------------------------------------------





// ------------------------------------------------------------------------
//                      allocate from exact bin  
//                      and below is controller
// ------------------------------------------------------------------------

//output address
assign exact_allocate_address = chunk_ptr_out+8;


// write signal
assign exact_write_addr = (EXACT_ST == EXACT_UNLINK_FD) ? alloc_chunk_info[2] + 12:
						  (EXACT_ST == EXACT_UNLINK_BK) ? alloc_chunk_info[3] +  8:
						   chunk_ptr_out + {alloc_chunk_info[1][31:1],1'b0} + 4;  //EXACT_ST == EXACT_SET_NEXTCHUNK
assign exact_write_data = (EXACT_ST == EXACT_UNLINK_FD) ? alloc_chunk_info[3] :
						  (EXACT_ST == EXACT_UNLINK_BK) ? alloc_chunk_info[2] :
						   {read_chunk_info[31:3],3'd1}; //EXACT_ST == EXACT_SET_NEXTCHUNK
assign exact_write_flag = (EXACT_ST == EXACT_UNLINK_FD & !last_fd) | (EXACT_ST == EXACT_UNLINK_BK & !last_bk);
assign exact_write_len = 1;


//read signal
assign exact_read_addr = (EXACT_ST == EXACT_SET_NEXTCHUNK) ? (chunk_ptr_out + {alloc_chunk_info[1][31:1],1'b0} + 4) : 
                          alloc_chunk_info[3]; //EXACT_ST == EXACT_UPDATE_NEW
assign exact_read_flag = (EXACT_ST == EXACT_SET_NEXTCHUNK) | (EXACT_ST == EXACT_UPDATE_NEW);
assign exact_rw_flag = EXACT_ST == EXACT_SET_NEXTCHUNK;
assign exact_read_len = (EXACT_ST == EXACT_SET_NEXTCHUNK) ? 3'd1 : 3'd4;


//others
assign last_fd = alloc_chunk_info[2] == 0;
assign last_bk = alloc_chunk_info[3] == 0;


/* record the next chunk size and modify previous in use bit */
always @(posedge clk)begin
  if(EXACT_ST == EXACT_SET_NEXTCHUNK & read_valid)
    read_chunk_info <= read_data;
end


always @(posedge clk)begin
  if(rst)
    EXACT_ST <= EXACT_IDLE;
  else
    EXACT_ST <= EXACT_NST;
end


always @(*)begin
  case(EXACT_ST)
     EXACT_IDLE:    
	   if(MAIN_ST == EXACT) EXACT_NST = EXACT_OUTPUT;
	   else EXACT_NST = TOP_IDLE;
	 EXACT_OUTPUT: //output allocate address and store chunk information from bin
	   if(share_cnt == 4) EXACT_NST = EXACT_UNLINK_FD;
	   else EXACT_NST = EXACT_OUTPUT;
	 EXACT_UNLINK_FD:
	   if(MEM_CTRL_ST == MEM_DONE | last_fd) EXACT_NST = EXACT_UNLINK_BK;
	   else EXACT_NST = EXACT_UNLINK_FD;
	 EXACT_UNLINK_BK:
	   if(MEM_CTRL_ST == MEM_DONE | last_bk) EXACT_NST = EXACT_SET_NEXTCHUNK;
	   else EXACT_NST = EXACT_UNLINK_BK;
	 EXACT_SET_NEXTCHUNK:
	   if(MEM_CTRL_ST == MEM_DONE) EXACT_NST = (last_bk) ? EXACT_UPDATE_EMPTY : EXACT_UPDATE_NEW;
	   else EXACT_NST = EXACT_SET_NEXTCHUNK;
	 EXACT_UPDATE_EMPTY:
	   if(share_cnt == 3) EXACT_NST = EXACT_DONE;
	   else EXACT_NST = EXACT_UPDATE_EMPTY;
	 EXACT_UPDATE_NEW:
	   if(MEM_CTRL_ST == MEM_DONE) EXACT_NST = EXACT_DONE;
	   else EXACT_NST = EXACT_UPDATE_NEW;
	 EXACT_DONE:
	   EXACT_NST = EXACT_IDLE;
	 default: 
	   EXACT_NST = EXACT_IDLE;
  endcase
end

// ------------------------------------------------------------------------
//                      allocate from exact bin end
// ------------------------------------------------------------------------









// ------------------------------------------------------------------------
//                     allocate from other(bigger) bin  
//                     and below is controller
// ------------------------------------------------------------------------

//output address
assign others_allocate_address = chunk_ptr_out+8;


//write signal
assign others_write_addr = (OTHERS_ST == OTHERS_SET_SIZE) ? chunk_ptr_out+4 : //fill previous in use bit in current size block
						   (OTHERS_ST == OTHERS_UNLINK_FD) ? alloc_chunk_info[2] + 12 :
                           (OTHERS_ST == OTHERS_UNLINK_BK) ? alloc_chunk_info[3] +  8 : 0;
assign others_write_data = (OTHERS_ST == OTHERS_SET_SIZE) ? (allocate_size_r | 32'h1)://fill previous in use bit in current size block
						   (OTHERS_ST == OTHERS_UNLINK_FD) ? alloc_chunk_info[3] :
                           (OTHERS_ST == OTHERS_UNLINK_BK) ? alloc_chunk_info[2] : 0;

assign others_write_flag = (OTHERS_ST == OTHERS_SET_SIZE) | (OTHERS_ST == OTHERS_UNLINK_FD & !last_fd) | (OTHERS_ST == OTHERS_UNLINK_BK & !last_bk);
assign others_write_len = 1;


/* read signal */	   
assign others_read_addr  = (OTHERS_ST == OTHERS_UPDATE_NEW) ? alloc_chunk_info[3] : 0; 		   
assign others_read_len   = 4;
assign others_read_flag = (OTHERS_ST == OTHERS_UPDATE_NEW);
						   
						  
always @(posedge clk)begin
  if(rst)
    OTHERS_ST <= OTHERS_IDLE;
  else
    OTHERS_ST <= OTHERS_NST;
end

always @(*)begin
  case(OTHERS_ST)
    OTHERS_IDLE:
	  if(MAIN_ST == OTHERS) OTHERS_NST = OTHERS_OUTPUT;
	  else OTHERS_NST = OTHERS_IDLE;
	OTHERS_OUTPUT:
      if(share_cnt == 4) OTHERS_NST = OTHERS_SET_SIZE;
	  else OTHERS_NST = OTHERS_OUTPUT;
	OTHERS_SET_SIZE:
	  if(MEM_CTRL_ST == MEM_DONE) OTHERS_NST = OTHERS_UNLINK_FD;
	  else OTHERS_NST = OTHERS_SET_SIZE;	
	OTHERS_UNLINK_FD:
	  if(MEM_CTRL_ST == MEM_DONE | last_fd) OTHERS_NST = OTHERS_UNLINK_BK;
	  else OTHERS_NST = OTHERS_UNLINK_FD;
	OTHERS_UNLINK_BK:
	  if(MEM_CTRL_ST == MEM_DONE | last_bk) OTHERS_NST = (last_bk) ? OTHERS_UPDATE_EMPTY : OTHERS_UPDATE_NEW;
	  else OTHERS_NST = OTHERS_UNLINK_BK;
	OTHERS_UPDATE_EMPTY:
	  if(share_cnt == 3) OTHERS_NST = OTHERS_DONE;
	  else OTHERS_NST = OTHERS_UPDATE_EMPTY;
	OTHERS_UPDATE_NEW: 
	  if(MEM_CTRL_ST == MEM_DONE) OTHERS_NST = OTHERS_DONE;
	  else OTHERS_NST = OTHERS_UPDATE_NEW;    
	OTHERS_DONE:
      OTHERS_NST = OTHERS_IDLE;
	default:
	  OTHERS_NST = OTHERS_IDLE;
  endcase
end

// ------------------------------------------------------------------------
//                      allocate from other bin end
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------
//                       insert controlloer begin
// ------------------------------------------------------------------------

assign insert_small_size = insert_size<512;


/* write signal */
assign insert_write_addr = (INSERT_ST == INSERT_LINK_BK & (!insert_smallest | insert_small_size)) ? current_pointer+8 :
                           (INSERT_ST == INSERT_SET_HEAD) ? insert_address+4 : //begin from size, FD, BK
						   (INSERT_ST == INSERT_SET_FOOT) ? insert_address+insert_size : //previous size of next chunk
						   (INSERT_ST == INSERT_LINK_BK & insert_smallest) ? current_pointer+12:
						   (INSERT_ST == INSERT_LINK_FD) ? alloc_chunk_info[2]+12 :
						   0;
						   
assign insert_write_data = (INSERT_ST == INSERT_LINK_BK) ? insert_address :
                           (INSERT_ST == INSERT_SET_HEAD) ? insert_chunk_info[share_cnt+1] :
						   (INSERT_ST == INSERT_SET_FOOT) ? {insert_chunk_info[1][31:1],1'b0} :
						   (INSERT_ST == INSERT_LINK_FD) ? insert_address :
						   0;
assign insert_write_flag = (INSERT_ST == INSERT_LINK_BK) | (INSERT_ST == INSERT_SET_HEAD) | (INSERT_ST == INSERT_SET_FOOT) | (INSERT_ST == INSERT_LINK_FD);
assign insert_write_len  = (INSERT_ST == INSERT_SET_HEAD) ? 2'd3 : 2'd1;
	  
	  
/* read signal */						   
assign insert_read_addr = (INSERT_ST == INSERT_READ_CHUNK) ? alloc_chunk_info[3] : 0;
assign insert_read_len  = (INSERT_ST == INSERT_READ_CHUNK) ? 3'd4 : 3'd0;
assign insert_read_flag = (INSERT_ST == INSERT_READ_CHUNK);


/* search insert position success */
assign search_success = {alloc_chunk_info[1][31:1],1'b0} <= insert_size;
/* insert free list is belong to the empty */
assign insert_not_empty_bin = bin_bitmap[bin_at_r];
/* insert free list is belong to the smallest */
assign insert_smallest = (alloc_chunk_info[3] ==  0 & {alloc_chunk_info[1][31:1],1'b0} > insert_size);


/* record and update insert chunk size */
always @(posedge clk)begin
  if(OTHERS_ST == OTHERS_OUTPUT) //split size
    insert_size <= {alloc_chunk_info[1][31:1],1'b0} - allocate_size_r;
  else if(FREE_ST == FREE_NTOP_DONE)
    insert_size <= {free_size_r[31:1],1'b0};
end


/* record and update insert chunk pointer */
always @(posedge clk)begin
  if(OTHERS_ST == OTHERS_OUTPUT) //split address
    insert_address <= chunk_ptr_out + allocate_size_r;
  else if(FREE_ST == FREE_NTOP_DONE)
    insert_address <= free_address_r;
end


/* record current alloc_chunk_info pointer */
always @(posedge clk)begin
  if(INSERT_ST == INSERT_READ_BIN)
    current_pointer <= chunk_ptr_out;
  else if(INSERT_ST == INSERT_ANALYSIS & !search_success & !insert_smallest)
    current_pointer <= alloc_chunk_info[3];
end


/* need update cache if the insert chunk->fd is in cache */
always @(posedge clk)begin
  if(INSERT_ST == INSERT_IDLE)
    insert_update_cache <= 0;
  else if(INSERT_ST == INSERT_ANALYSIS) 
    insert_update_cache <= (alloc_chunk_info[2] == 0 & alloc_chunk_info[3] == 0 & insert_smallest) | (chunk_ptr_out == alloc_chunk_info[2] & !insert_smallest);
  else if(INSERT_ST == INSERT_LINK_BK)
    insert_update_cache <= 0;
end


/* record and update insert chunk information */
always @(posedge clk)begin
  if(INSERT_ST == INSERT_IDLE)begin
    insert_chunk_info[0] <= 0;
    insert_chunk_info[1] <= insert_size | 1'b1;
    insert_chunk_info[2] <= 0 ;
    insert_chunk_info[3] <= 0;	
  end 
  else if(INSERT_ST == INSERT_READ_BIN)begin
    insert_chunk_info[0] <= 0;
    insert_chunk_info[1] <= insert_size | 1'b1;
    insert_chunk_info[2] <= 0 ;
    insert_chunk_info[3] <= (insert_not_empty_bin) ? chunk_ptr_out : 0;	
  end 
  else if(INSERT_ST == INSERT_ANALYSIS)begin
    if(search_success)begin
      insert_chunk_info[2] <= alloc_chunk_info[2];
      insert_chunk_info[3] <= current_pointer;	  
	end
    else if(insert_smallest)begin
      insert_chunk_info[2] <= current_pointer;
      insert_chunk_info[3] <= 0;	  
	end
	  
  end
end


/* insert state machine */
always @(posedge clk)begin
  if(rst)
    INSERT_ST <= INSERT_IDLE;
  else
    INSERT_ST <= INSERT_NST;
end


always @(*)begin
  case(INSERT_ST)
    INSERT_IDLE:
      if(MAIN_ST == INSERT)INSERT_NST = INSERT_DELAY;
	  else INSERT_NST = INSERT_IDLE;
    INSERT_DELAY:
	  INSERT_NST = INSERT_READ_BIN;
    INSERT_READ_BIN:
      if(share_cnt == 4) begin	    
	    if(insert_not_empty_bin) INSERT_NST = (insert_small_size) ?  INSERT_LINK_BK : INSERT_ANALYSIS;
	    else INSERT_NST = INSERT_SET_HEAD;
	  end
	  else INSERT_NST = INSERT_READ_BIN;
    INSERT_ANALYSIS:
	  if(search_success)INSERT_NST = INSERT_LINK_FD;
	  else if(insert_smallest) INSERT_NST = INSERT_LINK_BK;
	  else INSERT_NST = INSERT_READ_CHUNK;
    INSERT_READ_CHUNK:
	  if(MEM_CTRL_ST == MEM_DONE) INSERT_NST = INSERT_ANALYSIS;
	  else INSERT_NST = INSERT_READ_CHUNK;  
    INSERT_LINK_FD: /* insert node -> FD  */
	  if(MEM_CTRL_ST == MEM_DONE | last_fd) INSERT_NST = INSERT_LINK_BK;
	  else INSERT_NST = INSERT_LINK_FD;  
    INSERT_LINK_BK: /* insert node -> BK  */
	  if(MEM_CTRL_ST == MEM_DONE) INSERT_NST = INSERT_SET_HEAD;
	  else INSERT_NST = INSERT_LINK_BK;     
    INSERT_SET_HEAD:
	  if(MEM_CTRL_ST == MEM_DONE) INSERT_NST = INSERT_SET_FOOT;
	  else INSERT_NST = INSERT_SET_HEAD;  
	INSERT_SET_FOOT: /* check whether update the bin. insert_chunk_info[2]=0 => insert chunk is last chunk */
	  if(MEM_CTRL_ST == MEM_DONE) INSERT_NST = (insert_chunk_info[2] == 32'd0) ? INSERT_UPDATE : INSERT_DONE;
	  else INSERT_NST = INSERT_SET_FOOT;  
    INSERT_UPDATE:
	  if(share_cnt == 3)INSERT_NST = INSERT_DONE;
	  else INSERT_NST = INSERT_UPDATE;
    INSERT_DONE:
      INSERT_NST = 	INSERT_IDLE;
	default:
	  INSERT_NST = INSERT_IDLE;
  endcase
end

// ------------------------------------------------------------------------
//                        insert controlloer end
// ------------------------------------------------------------------------



// ------------------------------------------------------------------------
//                        free controlloer begin
// ------------------------------------------------------------------------


/* read signal */
assign free_read_addr = (FREE_ST == FREE_READ_SIZE)     ? free_address_r :    //4
                        (FREE_ST == FREE_READ_PREVIOUS) ? free_address_r :  //previous addresss
                        (FREE_ST == FREE_READ_NEXT)     ? free_address_r + {free_size_r[31:1],1'b0} :
                        (FREE_ST == FREE_READ_USEBIT)   ? (free_address_r + {free_size_r[31:1],1'b0} + {alloc_chunk_info[1][31:1],1'b0})+4 ://next->next.size
						(FREE_ST == FREE_UPDATE_NEW)    ? alloc_chunk_info[3] :
						0;
						
assign free_read_flag  = (FREE_ST == FREE_READ_SIZE) |(FREE_ST == FREE_READ_PREVIOUS) |(FREE_ST == FREE_READ_NEXT) |(FREE_ST == FREE_READ_USEBIT) | (FREE_ST == FREE_UPDATE_NEW);
assign free_read_len = (FREE_ST == FREE_READ_SIZE | FREE_ST == FREE_READ_PREVIOUS || FREE_ST == FREE_READ_NEXT || FREE_ST == FREE_UPDATE_NEW) ? 3'd4 : 3'd1;
                       

					   
/* write signal */
assign free_write_addr  = (FREE_ST == FREE_UNLINK_FD)    ? alloc_chunk_info[2] + 12: //1
						  (FREE_ST == FREE_UNLINK_BK)    ? alloc_chunk_info[3] +  8: //1		
						  (FREE_ST == FREE_CLEAR_USEBIT) ? free_address_r + {free_size_r[31:1],1'b0}+4 :		
						  (FREE_ST == FREE_INITIAL) ? free_address_r + 4 :		
                           0;						  
assign free_write_data  = (FREE_ST == FREE_UNLINK_FD)    ?  alloc_chunk_info[3] :  
						  (FREE_ST == FREE_UNLINK_BK)    ?  alloc_chunk_info[2] :
						  (FREE_ST == FREE_CLEAR_USEBIT) ? {alloc_chunk_info[1][31:1],1'b0} :
						  (FREE_ST == FREE_INITIAL)      ? 0 :
                           0;						  
assign free_write_flag = (FREE_ST == FREE_UNLINK_FD & !last_fd) |(FREE_ST == FREE_UNLINK_BK & !last_bk) |(FREE_ST == FREE_CLEAR_USEBIT) |(FREE_ST == FREE_INITIAL) ;
assign free_write_len  = (FREE_ST == FREE_UNLINK_FD  | FREE_ST == FREE_UNLINK_BK) |(FREE_ST == FREE_CLEAR_USEBIT) |(FREE_ST == FREE_INITIAL) ? 3'd1 : 3'd4;


/* next chunk is in the top */
	  
assign in_top = (free_address_r + {free_size_r[31:1],1'b0} + 4) == top_addr;
/* next_chunk->next is in the top and do not need to read next_chunk use bit  */

assign next_in_top = (FREE_ST == FREE_CLEAR_USEBIT) & (({alloc_chunk_info[1][31:1],1'b0}+free_address_r + {free_size_r[31:1],1'b0}+4)==top_addr);

/* previous address */
assign previous_address = free_address_r - alloc_chunk_info[0];


/* free size and free address controller*/
always @(posedge clk)begin
  if(FREE_ST == FREE_IDLE)begin
    free_size_r <= 0;
	free_address_r <= free_addr-8;
  end
  else if(FREE_ST == FREE_READ_SIZE & MEM_CTRL_ST == MEM_DONE)
    free_size_r <= alloc_chunk_info[1];
  else if(FREE_ST == FREE_ANALYSIS & free_size_r[0] == 0)begin
    free_address_r <= previous_address;
	free_size_r <= free_size_r + {alloc_chunk_info[0][31:1],1'b1};//add previous size and previous->previous chunk must allocated
  end
  else if(FREE_ST == FREE_READ_USEBIT & read_valid & !read_data[0])//previous_inuse = 0 => merge next chunk
    free_size_r <= free_size_r + {alloc_chunk_info[1][31:1],1'b0};
end

/* record previous inuse bit */
always @(posedge clk)begin
  if(FREE_ST == FREE_IDLE)
    previous_inuse <= 0;
  else if(FREE_ST == FREE_READ_USEBIT & read_valid)
    previous_inuse <= read_data[0];
end


/* record top check has been checked out */
always @(posedge clk)begin
  if(FREE_ST == FREE_IDLE)
    top_check_complete <= 0;
  else if(FREE_ST == FREE_CHECK_TOP)
    top_check_complete <= 1;
end


/* if unlink chunk is in bin cache , need to update the bin cache */
always @(posedge clk)begin
  if(FREE_ST == FREE_IDLE)
    update_fd_cache <= 0;
  else if(FREE_ST == FREE_UNLINK_FD)
    update_fd_cache <= (chunk_ptr_out == alloc_chunk_info[2]) ? 1'b1 : 1'b0;
  else if(FREE_ST == FREE_UNLINK_BK)
    update_fd_cache <= 0;
end


/* free state machine */
always @(posedge clk)begin
  if(rst)
    FREE_ST <= FREE_IDLE;
  else 
    FREE_ST <= FREE_NST;
end

always @(*)begin
  case(FREE_ST)
    FREE_IDLE:
	  if(MAIN_ST == FREE) FREE_NST = FREE_READ_SIZE; /* start free */
	  else FREE_NST = FREE_IDLE;
	FREE_READ_SIZE:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = FREE_INITIAL;
	  else FREE_NST = FREE_READ_SIZE;
	FREE_INITIAL:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = FREE_ANALYSIS;
	  else FREE_NST = FREE_INITIAL;	  
	FREE_ANALYSIS:
	  if(free_size_r[0] == 0) FREE_NST = FREE_READ_PREVIOUS;
	  else FREE_NST = FREE_CHECK_TOP;
	FREE_READ_PREVIOUS:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = FREE_UNLINK_FD;
	  else FREE_NST = FREE_READ_PREVIOUS;	
	FREE_UNLINK_FD:
	  if(MEM_CTRL_ST == MEM_DONE | last_fd) FREE_NST = FREE_UNLINK_BK;
	  else FREE_NST = FREE_UNLINK_FD;
	FREE_UNLINK_BK:
	  if(MEM_CTRL_ST == MEM_DONE | last_bk)begin
	    if(last_fd & last_bk) FREE_NST = FREE_UPDATE_EMPTY;
		else if(last_fd) FREE_NST = FREE_UPDATE_NEW;
		else if(top_check_complete) FREE_NST = FREE_NTOP_DONE;
		else FREE_NST = FREE_CHECK_TOP;
	  end
	  else FREE_NST = FREE_UNLINK_BK;
	FREE_CHECK_TOP:
	  if(in_top) FREE_NST = FREE_UPDATE_TOP;
	  else FREE_NST = FREE_READ_NEXT;
	FREE_READ_NEXT:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = FREE_CLEAR_USEBIT;
	  else FREE_NST = FREE_READ_NEXT;
	FREE_CLEAR_USEBIT:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = (next_in_top) ? FREE_NTOP_DONE : FREE_READ_USEBIT;
	  else FREE_NST = FREE_CLEAR_USEBIT;	  
	FREE_READ_USEBIT:
	  if(MEM_CTRL_ST == MEM_DONE)FREE_NST = (previous_inuse) ? FREE_NTOP_DONE : FREE_UNLINK_FD;
	  else FREE_NST = FREE_READ_USEBIT;	
	FREE_UPDATE_EMPTY:
	  if(share_cnt == 3) FREE_NST = (top_check_complete) ? FREE_NTOP_DONE : FREE_CHECK_TOP;
	  else FREE_NST = FREE_UPDATE_EMPTY;
	FREE_UPDATE_NEW: 
	  if(MEM_CTRL_ST == MEM_DONE) FREE_NST = (top_check_complete) ? FREE_NTOP_DONE : FREE_CHECK_TOP;
	  else FREE_NST = FREE_UPDATE_NEW;    
	FREE_UPDATE_TOP:
	  FREE_NST = FREE_TOP_DONE;
    FREE_NTOP_DONE:
	  FREE_NST = FREE_IDLE;
    FREE_TOP_DONE:
	  FREE_NST = FREE_IDLE;
	default:
	  FREE_NST = FREE_IDLE;
  endcase
end
  
  
// ------------------------------------------------------------------------
//                      free controlloer end
// ------------------------------------------------------------------------

/*
  bin block cache
  bin0:top 
  bin1:remainder(not use)
  bin2~bin127:cache each bin for last chunk
  chunk-> +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |             Size of previous chunk                            |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   head:' |             Size of chunk, in bytes       |previous in use bit|
    mem-> +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |             Forward pointer to next chunk in list             |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          |             Back pointer to previous chunk in list            |
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ 
*/
//bin cache for last chunk of each bin
sram_dual_port 
#(
  .RAM_WIDTH(32),
  .RAM_ADDR_BITS(10)
)
 bin_cache(
 //port A is chunk information
  .clka(clk),
  .we_A(bin_we),
  .en_A(1'b1),
  .addr_A(bin_addr),
  .data_iA(bin_data_in),
  .data_oA(bin_data_out),
  
 //port B is chunk pointer
  .clkb(clk),
  .we_B(chunk_ptr_we),
  .en_B(1'b1),
  .addr_B(chunk_ptr_addr),
  .data_iB(chunk_ptr_in),
  .data_oB(chunk_ptr_out)
);
// ------------------------------------------------------------------------
//align with allocate_size for multiples of 8 and the smallest size is 16 bytes
//area : 69 LUT
// ------------------------------------------------------------------------
assign allocate_size_tmp = allocate_size + 4;

always @(posedge clk)begin
  if(MAIN_ST == IDLE)begin
    if(allocate_size_tmp<16)
      allocate_size_align <= 16;
    else if(allocate_size_tmp[2:0] == 0)
      allocate_size_align <= allocate_size_tmp;
    else 
      allocate_size_align <= ((allocate_size_tmp+8) & 32'hFFFFFFF8);//multiples of 8
  end
  else if(MAIN_ST == INSERT)//priority state may change
    allocate_size_align <= insert_size;
  else if(MAIN_ST == FREE)/* (FREE_ST == FREE_READ_PREVIOUS || (FREE_ST == FREE_READ_USEBIT & MEM_CTRL_ST == MEM_DONE & previous_inuse) */
    allocate_size_align <= {alloc_chunk_info[1][31:1],1'b0};
end


// ------------------------------------------------------------------------
//find allocate size or free size that correspond bin
//area:104 LUT
// ------------------------------------------------------------------------
always @(*)begin
  bin_at =                                                     
    (((allocate_size_align >> 9) ==    0) ?       (allocate_size_align >>  3): 
     ((allocate_size_align >> 9) <=    4) ?  7'd56  + (allocate_size_align >>  6): 
     ((allocate_size_align >> 9) <=   20) ?  7'd91  + (allocate_size_align >>  9): 
     ((allocate_size_align >> 9) <=   84) ?  7'd110 + (allocate_size_align >> 12): 
     ((allocate_size_align >> 9) <=  340) ?  7'd119 + (allocate_size_align >> 15): 
     ((allocate_size_align >> 9) <= 1364) ?  7'd124 + (allocate_size_align >> 18): 
                                             7'd126);
end


always @(posedge clk)begin
  if(MAIN_ST == INIT)
    bin_bitmap <= 0;
  else if(EXACT_ST == EXACT_UPDATE_EMPTY)
    bin_bitmap[bin_at_r] <= 0;
  else if(OTHERS_ST == OTHERS_UPDATE_EMPTY)
    bin_bitmap[bin_at_r] <= 0;
  else if(FREE_ST == FREE_UPDATE_EMPTY)
    bin_bitmap[bin_at_r ]  <= 0;
  else if(INSERT_ST == INSERT_UPDATE)
    bin_bitmap[bin_at_r] <= 1;
end


// ------------------------------------
//bin mask
//area : 140 LUT
// ------------------------------------
always @(*)begin
  for(i=0;i<128;i=i+1)begin
    bin_mask[i] = (i>bin_at_r) ? 1'b1 : 1'b0;
  end
end


always @(*)begin
  bin_bitmap_tmp = bin_bitmap & bin_mask;
end

// ------------------------------------
//find fit bin
//area : 140 LUT
// ------------------------------------
always @(*)begin
         if (bin_bitmap_tmp[0  ])  search_bin_idx  =  0  ;
	else if (bin_bitmap_tmp[1  ])  search_bin_idx  =  1  ;
	else if (bin_bitmap_tmp[2  ])  search_bin_idx  =  2  ;
	else if (bin_bitmap_tmp[3  ])  search_bin_idx  =  3  ;
	else if (bin_bitmap_tmp[4  ])  search_bin_idx  =  4  ;
	else if (bin_bitmap_tmp[5  ])  search_bin_idx  =  5  ;
	else if (bin_bitmap_tmp[6  ])  search_bin_idx  =  6  ;
	else if (bin_bitmap_tmp[7  ])  search_bin_idx  =  7  ;
	else if (bin_bitmap_tmp[8  ])  search_bin_idx  =  8  ;
	else if (bin_bitmap_tmp[9  ])  search_bin_idx  =  9  ;
	else if (bin_bitmap_tmp[10 ])  search_bin_idx  =  10 ;
	else if (bin_bitmap_tmp[11 ])  search_bin_idx  =  11 ;
	else if (bin_bitmap_tmp[12 ])  search_bin_idx  =  12 ;
	else if (bin_bitmap_tmp[13 ])  search_bin_idx  =  13 ;
	else if (bin_bitmap_tmp[14 ])  search_bin_idx  =  14 ;
	else if (bin_bitmap_tmp[15 ])  search_bin_idx  =  15 ;
	else if (bin_bitmap_tmp[16 ])  search_bin_idx  =  16 ;
	else if (bin_bitmap_tmp[17 ])  search_bin_idx  =  17 ;
	else if (bin_bitmap_tmp[18 ])  search_bin_idx  =  18 ;
	else if (bin_bitmap_tmp[19 ])  search_bin_idx  =  19 ;
	else if (bin_bitmap_tmp[20 ])  search_bin_idx  =  20 ;
	else if (bin_bitmap_tmp[21 ])  search_bin_idx  =  21 ;
	else if (bin_bitmap_tmp[22 ])  search_bin_idx  =  22 ;
	else if (bin_bitmap_tmp[23 ])  search_bin_idx  =  23 ;
	else if (bin_bitmap_tmp[24 ])  search_bin_idx  =  24 ;
	else if (bin_bitmap_tmp[25 ])  search_bin_idx  =  25 ;
	else if (bin_bitmap_tmp[26 ])  search_bin_idx  =  26 ;
	else if (bin_bitmap_tmp[27 ])  search_bin_idx  =  27 ;
	else if (bin_bitmap_tmp[28 ])  search_bin_idx  =  28 ;
	else if (bin_bitmap_tmp[29 ])  search_bin_idx  =  29 ;
	else if (bin_bitmap_tmp[30 ])  search_bin_idx  =  30 ;
	else if (bin_bitmap_tmp[31 ])  search_bin_idx  =  31 ;
	else if (bin_bitmap_tmp[32 ])  search_bin_idx  =  32 ;
	else if (bin_bitmap_tmp[33 ])  search_bin_idx  =  33 ;
	else if (bin_bitmap_tmp[34 ])  search_bin_idx  =  34 ;
	else if (bin_bitmap_tmp[35 ])  search_bin_idx  =  35 ;
	else if (bin_bitmap_tmp[36 ])  search_bin_idx  =  36 ;
	else if (bin_bitmap_tmp[37 ])  search_bin_idx  =  37 ;
	else if (bin_bitmap_tmp[38 ])  search_bin_idx  =  38 ;
	else if (bin_bitmap_tmp[39 ])  search_bin_idx  =  39 ;
	else if (bin_bitmap_tmp[40 ])  search_bin_idx  =  40 ;
	else if (bin_bitmap_tmp[41 ])  search_bin_idx  =  41 ;
	else if (bin_bitmap_tmp[42 ])  search_bin_idx  =  42 ;
	else if (bin_bitmap_tmp[43 ])  search_bin_idx  =  43 ;
	else if (bin_bitmap_tmp[44 ])  search_bin_idx  =  44 ;
	else if (bin_bitmap_tmp[45 ])  search_bin_idx  =  45 ;
	else if (bin_bitmap_tmp[46 ])  search_bin_idx  =  46 ;
	else if (bin_bitmap_tmp[47 ])  search_bin_idx  =  47 ;
	else if (bin_bitmap_tmp[48 ])  search_bin_idx  =  48 ;
	else if (bin_bitmap_tmp[49 ])  search_bin_idx  =  49 ;
	else if (bin_bitmap_tmp[50 ])  search_bin_idx  =  50 ;
	else if (bin_bitmap_tmp[51 ])  search_bin_idx  =  51 ;
	else if (bin_bitmap_tmp[52 ])  search_bin_idx  =  52 ;
	else if (bin_bitmap_tmp[53 ])  search_bin_idx  =  53 ;
	else if (bin_bitmap_tmp[54 ])  search_bin_idx  =  54 ;
	else if (bin_bitmap_tmp[55 ])  search_bin_idx  =  55 ;
	else if (bin_bitmap_tmp[56 ])  search_bin_idx  =  56 ;
	else if (bin_bitmap_tmp[57 ])  search_bin_idx  =  57 ;
	else if (bin_bitmap_tmp[58 ])  search_bin_idx  =  58 ;
	else if (bin_bitmap_tmp[59 ])  search_bin_idx  =  59 ;
	else if (bin_bitmap_tmp[60 ])  search_bin_idx  =  60 ;
	else if (bin_bitmap_tmp[61 ])  search_bin_idx  =  61 ;
	else if (bin_bitmap_tmp[62 ])  search_bin_idx  =  62 ;
	else if (bin_bitmap_tmp[63 ])  search_bin_idx  =  63 ;
	else if (bin_bitmap_tmp[64 ])  search_bin_idx  =  64 ;
	else if (bin_bitmap_tmp[65 ])  search_bin_idx  =  65 ;
	else if (bin_bitmap_tmp[66 ])  search_bin_idx  =  66 ;
	else if (bin_bitmap_tmp[67 ])  search_bin_idx  =  67 ;
	else if (bin_bitmap_tmp[68 ])  search_bin_idx  =  68 ;
	else if (bin_bitmap_tmp[69 ])  search_bin_idx  =  69 ;
	else if (bin_bitmap_tmp[70 ])  search_bin_idx  =  70 ;
	else if (bin_bitmap_tmp[71 ])  search_bin_idx  =  71 ;
	else if (bin_bitmap_tmp[72 ])  search_bin_idx  =  72 ;
	else if (bin_bitmap_tmp[73 ])  search_bin_idx  =  73 ;
	else if (bin_bitmap_tmp[74 ])  search_bin_idx  =  74 ;
	else if (bin_bitmap_tmp[75 ])  search_bin_idx  =  75 ;
	else if (bin_bitmap_tmp[76 ])  search_bin_idx  =  76 ;
	else if (bin_bitmap_tmp[77 ])  search_bin_idx  =  77 ;
	else if (bin_bitmap_tmp[78 ])  search_bin_idx  =  78 ;
	else if (bin_bitmap_tmp[79 ])  search_bin_idx  =  79 ;
	else if (bin_bitmap_tmp[80 ])  search_bin_idx  =  80 ;
	else if (bin_bitmap_tmp[81 ])  search_bin_idx  =  81 ;
	else if (bin_bitmap_tmp[82 ])  search_bin_idx  =  82 ;
	else if (bin_bitmap_tmp[83 ])  search_bin_idx  =  83 ;
	else if (bin_bitmap_tmp[84 ])  search_bin_idx  =  84 ;
	else if (bin_bitmap_tmp[85 ])  search_bin_idx  =  85 ;
	else if (bin_bitmap_tmp[86 ])  search_bin_idx  =  86 ;
	else if (bin_bitmap_tmp[87 ])  search_bin_idx  =  87 ;
	else if (bin_bitmap_tmp[88 ])  search_bin_idx  =  88 ;
	else if (bin_bitmap_tmp[89 ])  search_bin_idx  =  89 ;
	else if (bin_bitmap_tmp[90 ])  search_bin_idx  =  90 ;
	else if (bin_bitmap_tmp[91 ])  search_bin_idx  =  91 ;
	else if (bin_bitmap_tmp[92 ])  search_bin_idx  =  92 ;
	else if (bin_bitmap_tmp[93 ])  search_bin_idx  =  93 ;
	else if (bin_bitmap_tmp[94 ])  search_bin_idx  =  94 ;
	else if (bin_bitmap_tmp[95 ])  search_bin_idx  =  95 ;
	else if (bin_bitmap_tmp[96 ])  search_bin_idx  =  96 ;
	else if (bin_bitmap_tmp[97 ])  search_bin_idx  =  97 ;
	else if (bin_bitmap_tmp[98 ])  search_bin_idx  =  98 ;
	else if (bin_bitmap_tmp[99 ])  search_bin_idx  =  99 ;
	else if (bin_bitmap_tmp[100])  search_bin_idx  =  100;
	else if (bin_bitmap_tmp[101])  search_bin_idx  =  101;
	else if (bin_bitmap_tmp[102])  search_bin_idx  =  102;
	else if (bin_bitmap_tmp[103])  search_bin_idx  =  103;
	else if (bin_bitmap_tmp[104])  search_bin_idx  =  104;
	else if (bin_bitmap_tmp[105])  search_bin_idx  =  105;
	else if (bin_bitmap_tmp[106])  search_bin_idx  =  106;
	else if (bin_bitmap_tmp[107])  search_bin_idx  =  107;
	else if (bin_bitmap_tmp[108])  search_bin_idx  =  108;
	else if (bin_bitmap_tmp[109])  search_bin_idx  =  109;
	else if (bin_bitmap_tmp[110])  search_bin_idx  =  110;
	else if (bin_bitmap_tmp[111])  search_bin_idx  =  111;
	else if (bin_bitmap_tmp[112])  search_bin_idx  =  112; 
	else if (bin_bitmap_tmp[113])  search_bin_idx  =  113;
	else if (bin_bitmap_tmp[114])  search_bin_idx  =  114;
	else if (bin_bitmap_tmp[115])  search_bin_idx  =  115;
	else if (bin_bitmap_tmp[116])  search_bin_idx  =  116;
	else if (bin_bitmap_tmp[117])  search_bin_idx  =  117;
	else if (bin_bitmap_tmp[118])  search_bin_idx  =  118;
	else if (bin_bitmap_tmp[119])  search_bin_idx  =  119;
	else if (bin_bitmap_tmp[120])  search_bin_idx  =  120;
	else if (bin_bitmap_tmp[121])  search_bin_idx  =  121;
	else if (bin_bitmap_tmp[122])  search_bin_idx  =  122;
	else if (bin_bitmap_tmp[123])  search_bin_idx  =  123;
	else if (bin_bitmap_tmp[124])  search_bin_idx  =  124;
	else if (bin_bitmap_tmp[125])  search_bin_idx  =  125;
	else if (bin_bitmap_tmp[126])  search_bin_idx  =  126;
	else if (bin_bitmap_tmp[127])  search_bin_idx  = 127;
	else search_bin_idx = 128;
end



endmodule