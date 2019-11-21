// TODO 旋轉 cnt 重複使用
module qr_decode(
input clk,                           //clock input
input srstn,                         //synchronous reset (active low)
input qr_decode_start,               //start decoding for one QR code
                                     //1: start (one-cycle pulse)

input sram_rdata,               //read data from SRAM
output reg [11:0] sram_raddr,         //read address to SRAM

output reg decode_valid,                 //decoded code is valid
output reg [7:0] decode_jis8_code,       //decoded JIS8 code
output reg qr_decode_finish          //1: decoding one QR code is finished
);

wire [7:0] a2i0_i, a2i1_i, a2i2_i, a2i3_i;
wire [7:0] i2a0_a, i2a1_a, i2a2_a, i2a3_a;
reg [7:0] a2i0_a, a2i1_a, a2i2_a, a2i3_a;
reg [7:0] i2a0_i, i2a1_i, i2a2_i, i2a3_i;

a2i a2i0(
	.a(a2i0_a),
	.i(a2i0_i)
);

a2i a2i1(
	.a(a2i1_a),
	.i(a2i1_i)
);

a2i a2i2(
	.a(a2i2_a),
	.i(a2i2_i)
);

a2i a2i3(
	.a(a2i3_a),
	.i(a2i3_i)
);

i2a i2a0(
	.i(i2a0_i),
	.a(i2a0_a)
);

i2a i2a1(
	.i(i2a1_i),
	.a(i2a1_a)
);

i2a i2a2(
	.i(i2a2_i),
	.a(i2a2_a)
);

i2a i2a3(
	.i(i2a3_i),
	.a(i2a3_a)
);

parameter IMG_LEN = 64, QR_LEN = 21;
parameter IDLE = 0, READ = 1, PROCESSING = 2,  ROTATE = 3, DECODING = 4, DEMASK=5, WRITE =6, FINISH = 7, READ_AFTER=8, SEARCH = 9, DECODING2 = 10;
parameter topleft = 0, bottomright = 1;
parameter MAIND = 0, OFFD= 1 , TOP= 2, LEFT = 3, TOP2 = 4, CHKR=5,CHKB=6, SEARCH_FINISH=7; 
reg qr_img_temp[QR_LEN-1:0][QR_LEN-1:0];
reg qr_img[QR_LEN-1:0][QR_LEN-1:0];
reg read_buffer[QR_LEN*QR_LEN:0];
wire corner[6:0][6:0];

reg [6:0] top_most_y, left_most_x;
reg [6:0] confirm_top_most_y, confirm_left_most_x;
reg [1:0] need_rotate_times;
reg [6:0] img_x;
reg [6:0] img_y;
reg corner_detect0;
reg corner_detect1;
reg corner_detect2;
reg corner_detect3;
reg [7:0] output_cnt;
reg [7:0] text_length;
reg searched_position;
assign corner[0][0] = 1;
assign corner[0][1] = 1;
assign corner[0][2] = 1;
assign corner[0][3] = 1;
assign corner[0][4] = 1;
assign corner[0][5] = 1;
assign corner[0][6] = 1;
assign corner[1][0] = 1;
assign corner[1][1] = 0;
assign corner[1][2] = 0;
assign corner[1][3] = 0;
assign corner[1][4] = 0;
assign corner[1][5] = 0;
assign corner[1][6] = 1;
assign corner[2][0] = 1;
assign corner[2][1] = 0;
assign corner[2][2] = 1;
assign corner[2][3] = 1;
assign corner[2][4] = 1;
assign corner[2][5] = 0;
assign corner[2][6] = 1;
assign corner[3][0] = 1;
assign corner[3][1] = 0;
assign corner[3][2] = 1;
assign corner[3][3] = 1;
assign corner[3][4] = 1;
assign corner[3][5] = 0;
assign corner[3][6] = 1;
assign corner[4][0] = 1;
assign corner[4][1] = 0;
assign corner[4][2] = 1;
assign corner[4][3] = 1;
assign corner[4][4] = 1;
assign corner[4][5] = 0;
assign corner[4][6] = 1;
assign corner[5][0] = 1;
assign corner[5][1] = 0;
assign corner[5][2] = 0;
assign corner[5][3] = 0;
assign corner[5][4] = 0;
assign corner[5][5] = 0;
assign corner[5][6] = 1;
assign corner[6][0] = 1;
assign corner[6][1] = 1;
assign corner[6][2] = 1;
assign corner[6][3] = 1;
assign corner[6][4] = 1;
assign corner[6][5] = 1;
assign corner[6][6] = 1;

reg [3:0] state;
reg [20:0] start_point_x;
reg [20:0] start_point_y;
reg img[IMG_LEN-1:0][IMG_LEN-1:0];
wire [3:0] x_corner1 = 14;
wire [3:0] y_corner2 = 14;
reg read_finished;
integer i, j, k, l;
reg rotate_finished;
reg [207:0]codewords;
reg [207+8:0]correct_codewords;
reg [1:0]rotate_cnt;
reg decoding_finish;


reg [8:0]S0;
reg [8:0]S1;
reg [8:0]S2;
reg [8:0]S3;
reg [8:0]S4;
reg [8:0]S5;

reg [8:0] alpha_array[25:0];
reg [8:0] Y1, Y2, Y1_a, Y2_a;
reg [2:0] search_state;
reg [9:0] search_cnt;

wire [7:0] x_s_array_0 = 1;
wire [7:0] x_s_array_1 = 4;
wire [7:0] x_s_array_2 = 16;
wire [7:0] x_s_array_3 = 64;
wire [7:0] x_s_array_4 = 29;
wire [7:0] x_s_array_5 = 116;
wire [7:0] x_s_array_6 = 205;
wire [7:0] x_s_array_7 = 19;
wire [7:0] x_s_array_8 = 76;
wire [7:0] x_s_array_9 = 45;
wire [7:0] x_s_array_10 = 180;
wire [7:0] x_s_array_11 = 234;
wire [7:0] x_s_array_12 = 143;
wire [7:0] x_s_array_13 = 6;
wire [7:0] x_s_array_14 = 24;
wire [7:0] x_s_array_15 = 96;
wire [7:0] x_s_array_16 = 157;
wire [7:0] x_s_array_17 = 78;
wire [7:0] x_s_array_18 = 37;
wire [7:0] x_s_array_19 = 148;
wire [7:0] x_s_array_20 = 106;
wire [7:0] x_s_array_21 = 181;
wire [7:0] x_s_array_22 = 238;
wire [7:0] x_s_array_23 = 159;
wire [7:0] x_s_array_24 = 70;
wire [7:0] x_s_array_25 = 5;
always @(posedge clk) begin
	if (!srstn) begin
		state <= IDLE;
		for(i=0; i<IMG_LEN; i=i+1) begin
			for(j=0; j<IMG_LEN; j=j+1) begin
				img[i][j] <= 0;
			end
		end
	end
	else if (qr_decode_start == 1'b1) begin
		state <= SEARCH;
	end
	else if (state == SEARCH) begin
		if (search_state == SEARCH_FINISH)
			state <= READ;
		else begin
			state <= state;
		end
	end
	else if (state == READ) begin
		if (read_finished) begin
			state <= READ_AFTER;
		end
		else begin
			state <= READ;
		end
	end
	else if (state == READ_AFTER) begin
		state <= PROCESSING;
	end
	else if (state == PROCESSING) begin
		state <= ROTATE;
	end
	else if (state == ROTATE) begin
		if (rotate_cnt == 0)
			state <= DEMASK;
		else begin
			state <= ROTATE;
		end
	end
	else if (state == DEMASK) begin
		state <= DECODING;
	end
	else if (state == DECODING) begin
		if (decoding_finish)
			state <= WRITE;
		else
			state <= DECODING;
	end
	else if (state == DECODING2) begin
		state <= WRITE;
	end
	else if (state == WRITE) begin
		if (output_cnt == text_length) begin
			// $display("\nS0_a = %d, S1_a = %d, S2_a = %d, S3_a = %d", S0_a, S1_a, S2_a, S3_a);
			// $display("ai1_a = %d, ai2_a = %d", ai1_a, ai2_a);
			// $display("alpha1_a = %d alpha2_a", alpha1_a, alpha2_a);
			// $display("Y1_a = %d, Y2_a = %d", Y1_a, Y2_a);
			// $display("offset1 = %d offset2 = %d", offset1, offset2);
		end
		if (output_cnt == text_length) begin
			state <= FINISH;
		end
		else begin
			state <= WRITE;
		end
	end
	else begin
		state <= state;
	end
end

always @* begin
	sram_raddr = img_x + ({img_y, 6'b0 });
end

always @(posedge clk) begin
	if (!srstn) begin
		img_x <= 10;
		img_y <= 10;
		search_state <= MAIND;
		search_cnt <= 0;
	end
	else if (state == SEARCH) begin
		if (search_state == MAIND) begin
			if(sram_rdata) begin
				search_state <= TOP;
				img_x <= img_x;
				img_y <= img_y;
				search_cnt <= 0;
			end
			else if (img_x > 50) begin
				search_state <= OFFD;
				img_x <= 63 - 10;
				img_y <= 0 	+ 10;
			end
			else begin
				img_x <= img_x + 1;
				img_y <= img_y + 1;
			end
		end
		else if (search_state == OFFD) begin
			if(sram_rdata) begin
				search_state <= TOP;
				img_x <= img_x;
				img_y <= img_y;
				search_cnt <= 0;
			end
			else if (img_y >= 63) begin
				$display("search fail");
				$finish();
			end
			else begin
				img_x <= img_x - 1;
				img_y <= img_y + 1;
			end
		end
		else if (search_state == TOP) begin
			if(search_cnt < 20 && img_y > 0) begin
				search_cnt <= search_cnt + 1;
				img_x <= img_x;
				img_y <= img_y - 1;
			end
			else begin
				search_cnt <= 0; 
				img_x <= img_x;
				img_y <= top_most_y;
				search_state <= LEFT;
			end
		end
		else if (search_state == LEFT) begin
			if(search_cnt < 20 && img_x > 0) begin
				search_cnt <= search_cnt + 1;
				img_x <= img_x - 1;
				img_y <= img_y;
			end
			else begin
				search_cnt <= 0; 
				img_x <= left_most_x;
				img_y <= top_most_y;
				search_state <= TOP2;
			end
		end
		else if (search_state == TOP2) begin
			if(search_cnt < 5 && img_y > 0) begin
				search_cnt <= search_cnt + 1;
				img_x <= img_x;
				img_y <= img_y - 1;
			end
			else begin
				search_cnt <= 0; 
				img_x <= left_most_x + 20;
				img_y <= top_most_y;
				search_state <= CHKR;
			end
		end
		else if(search_state == CHKR) begin
			// find right successfully, check b
			if (search_cnt == 0 && sram_rdata) begin
				confirm_left_most_x <= left_most_x;
				img_x <= left_most_x;
				img_y <= top_most_y + 20;
				search_state <= CHKB;
				search_cnt <= 0;
			end
			// find right fail, check 
			else if (search_cnt == 0 && !sram_rdata) begin
				search_cnt <= 1;
				img_x <= img_x;
				search_state <= CHKR;
				img_y <= top_most_y + 20;
			end 
			// find corner successfully
			else if (search_cnt == 1 && sram_rdata) begin
				confirm_left_most_x <= img_x - 20;
				confirm_top_most_y <= img_y - 20;
				img_x <= left_most_x;
				img_y <= top_most_y;
				search_state <= SEARCH_FINISH;
			end
			// find corner failt, turn to find right
			else if (search_cnt == 1 && !sram_rdata) begin
				img_x <= img_x - 1;
				img_y <= img_y - 20;
				search_cnt <= 3;
			end
			else if (sram_rdata) begin
				search_cnt <= 0;
				confirm_left_most_x <= img_x - 20;
				img_y <= top_most_y + 20;
				img_x <= img_x - 20;
				search_state <= CHKB;
			end
			else begin
				search_state <= CHKR;
				search_cnt <= search_cnt + 1;
				img_y <= img_y;
				img_x <= img_x - 1;
			end
		end
		else if (search_state == CHKB) begin
			// find buttom successfully
			if (search_cnt == 0 && sram_rdata) begin
				img_x <= confirm_left_most_x;
				img_y <= img_y - 20;
				confirm_top_most_y <= img_y - 20;
				search_state <= SEARCH_FINISH;
				search_cnt <= 0;
			end
			// try to find corner
			else if (search_cnt == 0 && !sram_rdata) begin
				img_x <= confirm_left_most_x + 20;
				img_y <= img_y;
				search_state <= CHKB;
				search_cnt <= 1;
			end
			else if (search_cnt == 1 && sram_rdata) begin
				img_x <= confirm_left_most_x;
				img_y <= img_y - 20;
				confirm_top_most_y <= img_y - 20;
				search_state <= SEARCH_FINISH;
				search_cnt <= 0;
			end
			// find corner fail
			else if (search_cnt == 1 && !sram_rdata) begin
				img_x <= confirm_left_most_x;
				img_y <= img_y - 1;
				search_state <= CHKB;
				search_cnt <= 2;
			end
			else if (sram_rdata) begin
				search_cnt <= 0;
				confirm_top_most_y <= img_y - 20;
				img_y <= img_y - 20;
				img_x <= img_x;
				search_state <= SEARCH_FINISH;
			end
			else begin
				search_state <= CHKB;
				search_cnt <= search_cnt + 1;
				img_y <= img_y-1;
				img_x <= img_x;
			end
		end
		else if (search_state == SEARCH_FINISH) begin
			search_cnt <= 0;
			img_y <= confirm_top_most_y;
			img_x <= confirm_left_most_x;
			search_state <= SEARCH_FINISH;
		end
	end
	else if (state == READ) begin
		
		search_cnt <= search_cnt + 1;
		if (search_cnt > 21*21 + 3)
			$finish();
		// $display("READ: img_x=%d img_y=%d, search_cnt=%d", img_x, img_y, search_cnt);
		if(img_x < confirm_left_most_x + QR_LEN -1) begin
			img_x <= img_x + 1;
			img_y <= img_y;
		end
		else begin
			img_x <= confirm_left_most_x; 
			img_y <= img_y + 1;
		end
	end
	else begin
	end
end

always @* begin
	read_finished = state == READ && img_y == confirm_top_most_y + QR_LEN;
end

always @(posedge clk) begin
	if (!srstn) begin
		qr_decode_finish <= 0;
	end
	else if (state == FINISH) begin
		qr_decode_finish <= 1;		
	end
	else begin
		qr_decode_finish <= 0;
	end
end

always @(posedge clk) begin
	if (!srstn) begin
		left_most_x <= ~0;
	end
	else if (sram_rdata && state == SEARCH && sram_rdata && img_x < left_most_x) begin
		left_most_x <= img_x;
	end
	else begin
		left_most_x <= left_most_x;
	end
end

always @(posedge clk) begin
	if (!srstn) begin
		top_most_y <= ~0;
	end
	else if (sram_rdata && state == SEARCH && sram_rdata && img_y < top_most_y) begin
		top_most_y <= img_y;
	end
	else begin
		top_most_y <= top_most_y;
	end
end

always @(posedge clk) begin
	if (!srstn) begin
		rotate_cnt <= ~0;
	end
	else if (state == READ_AFTER) begin
		rotate_cnt <= need_rotate_times;
	end
	else if (rotate_cnt > 0 && state == ROTATE) begin
		rotate_cnt <= rotate_cnt - 1; 
	end
	else begin
		rotate_cnt <= rotate_cnt;
	end
end

wire [2:0] mask_pattern = {qr_img[8][2], qr_img[8][3], qr_img[8][4]} ^ 3'b101;
always @(posedge clk) begin
	if (!srstn) begin
		for(i=0; i<QR_LEN; i=i+1) begin
			for (j=0; j<QR_LEN; j=j+1) begin
				qr_img[i][j] <= 0;				
			end
		end				
	end
	else if(state == READ) begin
		qr_img[img_y - confirm_top_most_y][img_x - confirm_left_most_x] <= sram_rdata;
	end
	// ROTATE
	else if (rotate_cnt > 0 && state == ROTATE) begin
		for (i = 0; i<21; i = i + 1) begin
			for (j=0; j<21; j= j+1) begin
				qr_img[j][20-i] <= qr_img[i][j];
			end
		end
	end
	else if (state == DEMASK) begin
		case(mask_pattern)
			3'b000: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ (((i+j)%2)==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
			end
			3'b001: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ (((i)%2)==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
				
			end
			3'b010: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ (((j)%3) == 0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
				
			end
			3'b011: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ (((i+j)%3)==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
				
			end
			3'b100: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ ((i/2+j/3)%2==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
				
			end
			3'b101: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ (((i*j)%2+((i*j)%3))==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
				
			end
			3'b110: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ ((((i*j)%2+((i*j)%3))%2)==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
			
			end
			3'b111: begin
				for(i=0; i<QR_LEN; i=i+1) begin
					for(j=0; j<QR_LEN; j=j+1) begin
						if (!	((0 <= i && i <= 8 && 0 <= j && j <= 8) ||
								(0 <= i && i <= 8 && 13 <= j && j <= 20)||
								(13 <= i && i <= 20 && 0 <= j && j <= 8)	)
						) begin
							qr_img[i][j] <= qr_img[i][j] ^ ((((i*j)%3+((i+j)%2))%2)==0); 
						end
						else begin
							qr_img[i][j] <= qr_img[i][j]; 							
						end
					end	
				end
	
			end
			
		endcase
	end
	else begin
		for(i=0; i<QR_LEN; i=i+1) begin
			for (j=0; j<QR_LEN; j=j+1) begin
				qr_img[i][j] <= qr_img[i][j];				
			end
		end						
	end
end

integer r0, c0;
always @* begin
	
	corner_detect0 = 1;
	corner_detect1 = 1;
	corner_detect2 = 1;

	for (r0 = 0; r0<3; r0=r0+1) begin
		for(c0=0; c0<3; c0=c0+1) begin
			if (qr_img[r0][c0] != corner[r0][c0]) begin
				corner_detect0 = 0;
			end
		end
	end


	for (r0 = 0; r0<3; r0=r0+1) begin
		for(c0=0; c0<3; c0=c0+1) begin
			if (qr_img[r0][c0+x_corner1] != corner[r0][c0]) begin
				corner_detect1 = 0;
			end
		end
	end


	for (r0 = 0; r0<3; r0=r0+1) begin
		for(c0=0; c0<3; c0=c0+1) begin
			if (qr_img[r0+y_corner2][c0] != corner[r0][c0]) begin
				corner_detect2 = 0;
			end
		end
	end

	if (corner_detect0 == 0) begin
		need_rotate_times = 2'b10;
	end
	else if (corner_detect1 == 0 ) begin
		need_rotate_times = 2'b01;		
	end
	else if (corner_detect2 == 0) begin
		need_rotate_times = 2'b11;		
	end
	else begin
		need_rotate_times = 2'b00;
	end
end

integer tempx,tempy, codewords_cnt;

always @* begin

	codewords_cnt = 0;

	
	tempy = 17;
	tempx = 19;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	
	tempy = 13;
	tempx = 19;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempy = 9;
	tempx = 19;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempy = 9;
	tempx = 17;

	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;

	tempy = 13;
	tempx = 17;

	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempy = 17;
	tempx = 17;

	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;

	tempx = 15;
	tempy = 17;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 15;
	tempy = 13;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 15;
	tempy = 9;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 13;
	tempy = 9;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 13;
	tempy = 13;



	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 13;
	tempy = 17;

	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;



	tempx = 11;
	tempy = 17;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 11;
	tempy = 13;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 11;
	tempy = 9;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 11;
	tempy = 4;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+4][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+4][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 11;
	tempy = 0;


	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 9;
	tempy = 0;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 9;
	tempy = 4;


	codewords[codewords_cnt] = qr_img[tempy+4][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+4][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 9;
	tempy = 9;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 9;
	tempy = 13;

	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 9;
	tempy = 17;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 7;
	tempy = 9;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;

	tempx = 4;
	tempy = 9;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;

	tempx = 2;
	tempy = 9;

	codewords[codewords_cnt] = qr_img[tempy][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;


	tempx = 0;
	tempy = 9;


	codewords[codewords_cnt] = qr_img[tempy+3][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+3][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+2][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+1][tempx+1];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx];
	codewords_cnt = codewords_cnt + 1;
	codewords[codewords_cnt] = qr_img[tempy+0][tempx+1];
	codewords_cnt = codewords_cnt + 1;
end

// OUTPUT RESULT

always @(posedge clk) begin
	if (!srstn) begin
		output_cnt <= 0;
	end
	else if (state == WRITE && output_cnt < text_length) begin
		output_cnt <= output_cnt + 1;
	end 
	else begin
		output_cnt <= output_cnt;		
	end
end

always @* begin
	text_length[7] = codewords[3];
	text_length[6] = codewords[2];
	text_length[5] = codewords[1];
	text_length[4] = codewords[0];
	text_length[3] = codewords[15];
	text_length[2] = codewords[14];
	text_length[1] = codewords[13];
	text_length[0] = codewords[12];
end

reg [7:0] decode_jis8_code_buf;
reg [7:0] decode_valid_buf;

always @(posedge clk) begin
	decode_valid <= decode_valid_buf;
	decode_jis8_code <= decode_jis8_code_buf;
end

always @* begin
	decode_valid_buf = state == WRITE && output_cnt < text_length;
	decode_jis8_code_buf[3:0] = correct_codewords[(output_cnt+2) * 8  +4  +:4];
	// decode_jis8_code_buf[3:0] = correct_codewords[{(output_cnt+2), 3'b0}  +4  +:4];
	decode_jis8_code_buf[7:4] = correct_codewords[(output_cnt+1) * 8	  +:4];
	// decode_jis8_code_buf[7:4] = correct_codewords[{(output_cnt+1), 3'b0} 	  +:4];
end


reg [8:0] sdiff_a;
reg [8:0] alpha1, alpha2;
reg [8:0] alpha1_a, alpha2_a;
reg [8:0] S0_a, S1_a, S2_a, S3_a, S4_a, S5_a;
reg [8:0] S6, S7, S6_a, S7_a, S8, S8_a;
reg [8:0] S9, S9_a;
reg [8:0] S10, S10_a;
reg [8:0] ai1, ai2;
reg [8:0] ai1_a, ai2_a;
reg findfirstai0, findfirstai1;
reg [8:0] temp_sum;
reg [8:0] a1, a2, b1, b2, c3, c4;
reg [8:0] c1, c2, c1_a, c2_a;
reg [8:0] b3, b3_a;
reg [8:0] a1_a, a2_a, b1_a, b2_a, c3_a, c4_a;
reg [8:0] ai1_s, ai2_s;
reg [8:0] ai1_s_a, ai2_s_a;
wire [8:0] diff_a = a1_a - a2_a;
reg [8:0] unknown_x, unknown_y;
reg [8:0] unknown_x_a, unknown_y_a;
reg [7:0] offset1, offset2;
reg [8:0] err_timer;
reg [7:0] offset1_t,offset2_t;
always @(posedge clk) begin
	if (!srstn) begin
		err_timer <= 0;
	end
	else if (state == DECODING) begin
		err_timer <= err_timer + 1; 
	end
	else begin
		err_timer <= err_timer;
	end
end

reg [8:0] S1_t;
reg [8:0] S2_t;
reg [8:0] S3_t;
reg [8:0] S4_t;
reg [8:0] b3_t, b2_a_t;
reg [8:0] b3_a_t,c2_a_t,c3_t, c3_a_t, alpha1_t, alpha1_a_t, c4_t, c4_a_t;
reg [8:0] alpha2_a_t, alpha2_t; 
reg [8:0] temp_sum_0;
reg [8:0] temp_sum_1;
reg [8:0] temp_sum_2;
reg [8:0] temp_sum_3;
reg [8:0] temp_sum_4;
reg [8:0] temp_sum_5;
reg [8:0] temp_sum_6;
reg [8:0] temp_sum_7;
reg [8:0] temp_sum_8;
reg [8:0] temp_sum_9;
reg [8:0] temp_sum_10;
reg [8:0] temp_sum_11;
reg [8:0] temp_sum_12;
reg [8:0] temp_sum_13;
reg [8:0] temp_sum_14;
reg [8:0] temp_sum_15;
reg [8:0] temp_sum_16;
reg [8:0] temp_sum_17;
reg [8:0] temp_sum_18;
reg [8:0] temp_sum_19;
reg [8:0] temp_sum_20;
reg [8:0] temp_sum_21;
reg [8:0] temp_sum_22;
reg [8:0] temp_sum_23;
reg [8:0] temp_sum_24;
reg [8:0] temp_sum_25;
reg [8:0] ai1_a_t;
reg [8:0] ai2_a_t;
reg findfirstai0_t, findfirstai1_t;
wire error_occur = S0 != 0 || S1 != 0 || S2 != 0 || S3 != 0;
reg [8:0] ai1_t, ai2_t, ai1_s_t, ai2_s_t;
reg [8:0] a1_a_t, a2_a_t;
reg [8:0] b1_a_t;
reg [8:0] Y1_a_t, Y2_a_t;
always @* begin
	i2a0_i = 0;
	i2a1_i = 0;
	i2a2_i = 0;
	i2a3_i = 0;
	a2i0_a = 0;
	a2i1_a = 0;
	a2i2_a = 0;
	a2i3_a = 0;
	// a b c d e f
	case(err_timer)
		0: begin
			i2a0_i = codewords[0*8 +:8];
			i2a1_i = codewords[1*8 +:8];
			i2a2_i = codewords[2*8 +:8];
			i2a3_i = codewords[3*8 +:8];
		end
		1: begin
			i2a0_i = codewords[4*8 +:8];
			i2a1_i = codewords[5*8 +:8];
			i2a2_i = codewords[6*8 +:8];
			i2a3_i = codewords[7*8 +:8];
			a2i0_a = (alpha_array[0] + 25-0) >= 255   ?(alpha_array[0] + 25-0) - 255: (alpha_array[0] + 25-0);
			a2i1_a = (alpha_array[1] + 25-1) >= 255   ?(alpha_array[1] + 25-1) - 255: (alpha_array[1] + 25-1);
			a2i2_a = (alpha_array[2] + 25-2) >= 255   ?(alpha_array[2] + 25-2) - 255: (alpha_array[2] + 25-2);
			a2i3_a = (alpha_array[3] + 25-3) >= 255   ?(alpha_array[3] + 25-3) - 255: (alpha_array[3] + 25-3);
			S1_t = a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;
		end
		2: begin
			i2a0_i = codewords[8*8  +:8];
			i2a1_i = codewords[9*8  +:8];
			i2a2_i = codewords[10*8 +:8];
			i2a3_i = codewords[11*8 +:8];
			a2i0_a = (alpha_array[4] + 25-4) >= 255   ?(alpha_array[4] + 25-4) - 255: (alpha_array[4] + 25-4);
			a2i1_a = (alpha_array[5] + 25-5) >= 255   ?(alpha_array[5] + 25-5) - 255: (alpha_array[5] + 25-5);
			a2i2_a = (alpha_array[6] + 25-6) >= 255   ?(alpha_array[6] + 25-6) - 255: (alpha_array[6] + 25-6);
			a2i3_a = (alpha_array[7] + 25-7) >= 255   ?(alpha_array[7] + 25-7) - 255: (alpha_array[7] + 25-7);
			S1_t = S1 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		3: begin
			i2a0_i = codewords[12*8  +:8];
			i2a1_i = codewords[13*8  +:8];
			i2a2_i = codewords[14*8 +:8];
			i2a3_i = codewords[15*8 +:8];
			a2i0_a = (alpha_array[8] + 25-8) >= 255   ?(alpha_array[8] + 25-8) - 255: (alpha_array[8] + 25-8);
			a2i1_a = (alpha_array[9] + 25-9) >= 255   ?(alpha_array[9] + 25-9) - 255: (alpha_array[9] + 25-9);
			a2i2_a = (alpha_array[10] + 25-10) >= 255   ?(alpha_array[10] + 25-10) - 255: (alpha_array[10] + 25-10);
			a2i3_a = (alpha_array[11] + 25-11) >= 255   ?(alpha_array[11] + 25-11) - 255: (alpha_array[11]+ 25-11);
			S1_t = S1 ^ a2i0_i^ a2i1_i ^ a2i2_i ^ a2i3_i;
		end
		4: begin

			i2a0_i = codewords[16*8  +:8];
			i2a1_i = codewords[17*8  +:8];
			i2a2_i = codewords[18*8 +:8];
			i2a3_i = codewords[19*8 +:8];
			a2i0_a = (alpha_array[12] + 25-12) >= 255    ?(alpha_array[12] + 25-12) - 255: (alpha_array[12] + 25-12);
			a2i1_a = (alpha_array[13] + 25-13) >= 255    ?(alpha_array[13] + 25-13) - 255: (alpha_array[13] + 25-13);
			a2i2_a = (alpha_array[14] + 25-14) >= 255    ?(alpha_array[14]+  25-14) - 255: (alpha_array[14] + 25-14);
			a2i3_a = (alpha_array[15] + 25-15) >= 255    ?(alpha_array[15]+  25-15) - 255: (alpha_array[15] + 25-15);
			S1_t = S1 ^ a2i0_i^ a2i1_i ^ a2i2_i ^ a2i3_i;
			
		end
		5: begin
			i2a0_i = codewords[20*8  +:8];
			i2a1_i = codewords[21*8  +:8];
			i2a2_i = codewords[22*8  +:8];
			i2a3_i = codewords[23*8  +:8];
			a2i0_a = (alpha_array[16] + 25-16) >= 255    ?(alpha_array[16] + 25-16) - 255: (alpha_array[16] + 25-16);
			a2i1_a = (alpha_array[17] + 25-17) >= 255    ?(alpha_array[17] + 25-17) - 255: (alpha_array[17] + 25-17);
			a2i2_a = (alpha_array[18] + 25-18) >= 255    ?(alpha_array[18]+  25-18) - 255: (alpha_array[18] + 25-18);
			a2i3_a = (alpha_array[19] + 25-19) >= 255    ?(alpha_array[19]+  25-19) - 255: (alpha_array[19] + 25-19);
			S1_t = S1 ^ a2i0_i^ a2i1_i ^ a2i2_i ^ a2i3_i;			
		end
		6: begin
			i2a0_i = codewords[24*8  +:8];
			i2a1_i = codewords[25*8  +:8];
			a2i0_a = (alpha_array[20] + 25-20) >= 255    ?(alpha_array[20] + 25-20) - 255: (alpha_array[20] + 25-20);
			a2i1_a = (alpha_array[21] + 25-21) >= 255    ?(alpha_array[21] + 25-21) - 255: (alpha_array[21] + 25-21);
			a2i2_a = (alpha_array[22] + 25-22) >= 255    ?(alpha_array[22]+  25-22) - 255: (alpha_array[22] + 25-22);
			a2i3_a = (alpha_array[23] + 25-23) >= 255    ?(alpha_array[23]+  25-23) - 255: (alpha_array[23] + 25-23);
			S1_t = S1 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;
		end
		7: begin
			a2i0_a = (alpha_array[24] + 25-24) >= 255    ?(alpha_array[24] + 25-24) - 255: (alpha_array[24] + 25-24);
			a2i1_a = (alpha_array[25] + 25-25) >= 255    ?(alpha_array[25] + 25-25) - 255: (alpha_array[25] + 25-25);
			S1_t = S1 ^ a2i0_i ^ a2i1_i;
		end
		8: begin

			a2i0_a = (alpha_array[0] + (25-0)*2) >= 255   ?(alpha_array[0] + (25-0)*2) - 255: (alpha_array[0] + (25-0)*2);
			a2i1_a = (alpha_array[1] + (25-1)*2) >= 255   ?(alpha_array[1] + (25-1)*2) - 255: (alpha_array[1] + (25-1)*2);
			a2i2_a = (alpha_array[2] + (25-2)*2) >= 255   ?(alpha_array[2] + (25-2)*2) - 255: (alpha_array[2] + (25-2)*2);
			a2i3_a = (alpha_array[3] + (25-3)*2) >= 255   ?(alpha_array[3] + (25-3)*2) - 255: (alpha_array[3] + (25-3)*2);
			S2_t = a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;
		end
		9: begin
			a2i0_a = (alpha_array[4] + (25-4)*2) >= 255   ?(alpha_array[4] + (25-4)*2) - 255: (alpha_array[4] + (25-4)*2);
			a2i1_a = (alpha_array[5] + (25-5)*2) >= 255   ?(alpha_array[5] + (25-5)*2) - 255: (alpha_array[5] + (25-5)*2);
			a2i2_a = (alpha_array[6] + (25-6)*2) >= 255   ?(alpha_array[6] + (25-6)*2) - 255: (alpha_array[6] + (25-6)*2);
			a2i3_a = (alpha_array[7] + (25-7)*2) >= 255   ?(alpha_array[7] + (25-7)*2) - 255: (alpha_array[7] + (25-7)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		10: begin
			a2i0_a = (alpha_array[8] + (25-8)   *2)>= 255 ?(alpha_array[8] + (25-8)*2) - 255: (alpha_array[8] + (25-8)*2);
			a2i1_a = (alpha_array[9] + (25-9)   *2)>= 255 ?(alpha_array[9] + (25-9)*2) - 255: (alpha_array[9] + (25-9)*2);
			a2i2_a = (alpha_array[10] + (25-10) *2)>= 255 ?(alpha_array[10] + (25-10)*2) - 255: (alpha_array[10] + (25-10)*2);
			a2i3_a = (alpha_array[11] + (25-11) *2)>= 255 ?(alpha_array[11] + (25-11)*2) - 255: (alpha_array[11]+ (25-11)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		11: begin
			a2i0_a = (alpha_array[12] + (25-12)*2) >= 255 ?(alpha_array[12] + (25-12)*2) - 255: (alpha_array[12] + (25-12)*2);
			a2i1_a = (alpha_array[13] + (25-13)*2) >= 255 ?(alpha_array[13] + (25-13)*2) - 255: (alpha_array[13] + (25-13)*2);
			a2i2_a = (alpha_array[14] + (25-14)*2) >= 255 ?(alpha_array[14] + (25-14)*2) - 255: (alpha_array[14] + (25-14)*2);
			a2i3_a = (alpha_array[15] + (25-15)*2) >= 255 ?(alpha_array[15] + (25-15)*2) - 255: (alpha_array[15] + (25-15)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		12: begin
			a2i0_a = (alpha_array[16] + (25-16)*2) >= 255 ?(alpha_array[16] + (25-16)*2) - 255: (alpha_array[16] + (25-16)*2);
			a2i1_a = (alpha_array[17] + (25-17)*2) >= 255 ?(alpha_array[17] + (25-17)*2) - 255: (alpha_array[17] + (25-17)*2);
			a2i2_a = (alpha_array[18] + (25-18)*2) >= 255 ?(alpha_array[18] + (25-18)*2) - 255: (alpha_array[18] + (25-18)*2);
			a2i3_a = (alpha_array[19] + (25-19)*2) >= 255 ?(alpha_array[19] + (25-19)*2) - 255: (alpha_array[19] + (25-19)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		13: begin
			a2i0_a = (alpha_array[20] + (25-20)*2) >= 255 ?(alpha_array[20] + (25-20)*2) - 255: (alpha_array[20] + (25-20)*2);
			a2i1_a = (alpha_array[21] + (25-21)*2) >= 255 ?(alpha_array[21] + (25-21)*2) - 255: (alpha_array[21] + (25-21)*2);
			a2i2_a = (alpha_array[22] + (25-22)*2) >= 255 ?(alpha_array[22] + (25-22)*2) - 255: (alpha_array[22] + (25-22)*2);
			a2i3_a = (alpha_array[23] + (25-23)*2) >= 255 ?(alpha_array[23] + (25-23)*2) - 255: (alpha_array[23] + (25-23)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		14: begin
			a2i0_a = (alpha_array[24] + (25-24)*2) >= 255    ?(alpha_array[24]+ ( 25-24*2)) - 255: (alpha_array[24] + (25-24)*2);
			a2i1_a = (alpha_array[25] + (25-25)*2) >= 255    ?(alpha_array[25]+ ( 25-25*2)) - 255: (alpha_array[25] + (25-25)*2);
			S2_t = S2 ^ a2i0_i ^ a2i1_i;
		end
		15:begin
			
			a2i0_a = (alpha_array[0] + (25-0)*3) >= 255   ?(alpha_array[0] + (25-0)*3) - 255: (alpha_array[0] + (25-0)*3);
			a2i1_a = (alpha_array[1] + (25-1)*3) >= 255   ?(alpha_array[1] + (25-1)*3) - 255: (alpha_array[1] + (25-1)*3);
			a2i2_a = (alpha_array[2] + (25-2)*3) >= 255   ?(alpha_array[2] + (25-2)*3) - 255: (alpha_array[2] + (25-2)*3);
			a2i3_a = (alpha_array[3] + (25-3)*3) >= 255   ?(alpha_array[3] + (25-3)*3) - 255: (alpha_array[3] + (25-3)*3);
			S3_t = a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		16: begin
			a2i0_a = (alpha_array[4] + (25-4)*3) >= 255   ?(alpha_array[4] + (25-4)*3) - 255: (alpha_array[4] + (25-4)*3);
			a2i1_a = (alpha_array[5] + (25-5)*3) >= 255   ?(alpha_array[5] + (25-5)*3) - 255: (alpha_array[5] + (25-5)*3);
			a2i2_a = (alpha_array[6] + (25-6)*3) >= 255   ?(alpha_array[6] + (25-6)*3) - 255: (alpha_array[6] + (25-6)*3);
			a2i3_a = (alpha_array[7] + (25-7)*3) >= 255   ?(alpha_array[7] + (25-7)*3) - 255: (alpha_array[7] + (25-7)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		17: begin
			a2i0_a = (alpha_array[8] + (25-8)   *3)>= 255 ?(alpha_array[8] + (25-8)*3) - 255: (alpha_array[8] + (25-8)*3);
			a2i1_a = (alpha_array[9] + (25-9)   *3)>= 255 ?(alpha_array[9] + (25-9)*3) - 255: (alpha_array[9] + (25-9)*3);
			a2i2_a = (alpha_array[10] + (25-10) *3)>= 255 ?(alpha_array[10] + (25-10)*3) - 255: (alpha_array[10] + (25-10)*3);
			a2i3_a = (alpha_array[11] + (25-11) *3)>= 255 ?(alpha_array[11] + (25-11)*3) - 255: (alpha_array[11]+ (25-11)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		18: begin
			a2i0_a = (alpha_array[12] + (25-12)*3) >= 255 ?(alpha_array[12] + (25-12)*3) - 255: (alpha_array[12] + (25-12)*3);
			a2i1_a = (alpha_array[13] + (25-13)*3) >= 255 ?(alpha_array[13] + (25-13)*3) - 255: (alpha_array[13] + (25-13)*3);
			a2i2_a = (alpha_array[14] + (25-14)*3) >= 255 ?(alpha_array[14] + (25-14)*3) - 255: (alpha_array[14] + (25-14)*3);
			a2i3_a = (alpha_array[15] + (25-15)*3) >= 255 ?(alpha_array[15] + (25-15)*3) - 255: (alpha_array[15] + (25-15)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		19: begin
			a2i0_a = (alpha_array[16] + (25-16)*3) >= 255 ?(alpha_array[16] + (25-16)*3) - 255: (alpha_array[16] + (25-16)*3);
			a2i1_a = (alpha_array[17] + (25-17)*3) >= 255 ?(alpha_array[17] + (25-17)*3) - 255: (alpha_array[17] + (25-17)*3);
			a2i2_a = (alpha_array[18] + (25-18)*3) >= 255 ?(alpha_array[18] + (25-18)*3) - 255: (alpha_array[18] + (25-18)*3);
			a2i3_a = (alpha_array[19] + (25-19)*3) >= 255 ?(alpha_array[19] + (25-19)*3) - 255: (alpha_array[19] + (25-19)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;

		end
		20: begin
			a2i0_a = (alpha_array[20] + (25-20)*3) >= 255 ?(alpha_array[20] + (25-20)*3) - 255: (alpha_array[20] + (25-20)*3);
			a2i1_a = (alpha_array[21] + (25-21)*3) >= 255 ?(alpha_array[21] + (25-21)*3) - 255: (alpha_array[21] + (25-21)*3);
			a2i2_a = (alpha_array[22] + (25-22)*3) >= 255 ?(alpha_array[22] + (25-22)*3) - 255: (alpha_array[22] + (25-22)*3);
			a2i3_a = (alpha_array[23] + (25-23)*3) >= 255 ?(alpha_array[23] + (25-23)*3) - 255: (alpha_array[23] + (25-23)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i ^ a2i2_i ^ a2i3_i;
			i2a0_i = S0;  
			i2a1_i = S1;  
			i2a2_i = S2; 
		end
		21: begin
			a2i0_a = (alpha_array[24] + (25-24)*3) >= 255    ?(alpha_array[24]+ ( 25-24*3)) - 255: (alpha_array[24] + (25-24)*3);
			a2i1_a = (alpha_array[25] + (25-25)*3) >= 255    ?(alpha_array[25]+ ( 25-25*3)) - 255: (alpha_array[25] + (25-25)*3);
			S3_t = S3 ^ a2i0_i ^ a2i1_i;
			i2a0_i = S3_t; 
		end
		22 :begin

		end
		23 :begin
			i2a0_i = a1;
			i2a1_i = a2;
			i2a2_i = b1;
			i2a3_i = b2;
			b2_a_t = i2a3_a;
			a2i0_a = (i2a3_a + (i2a0_a-i2a1_a)) >= 255 ? (i2a3_a + (i2a0_a-i2a1_a)) - 255 : (i2a3_a + (i2a0_a-i2a1_a));
			b3_t = b1 ^ a2i0_i;
		end
		24 :begin
			i2a0_i = b3;
			i2a1_i = c2;
			b3_a_t = i2a0_a;
			c2_a_t = i2a1_a;
			a2i0_a = (c2_a_t + diff_a) >= 255 ? (c2_a_t + diff_a) - 255 : (c2_a_t + diff_a);
			c3_t = c1 ^ a2i0_i;
			i2a2_i = c3_t;
			c3_a_t = i2a2_a;
			alpha1_a_t = (c3_a_t + 255 - b3_a_t) >= 255 ?(c3_a_t + 255 - b3_a_t) - 255:(c3_a_t + 255 - b3_a_t);
			a2i1_a = alpha1_a_t;
			alpha1_t = a2i1_i;
			a2i2_a = (b1_a + alpha1_a_t) >= 255 ? (b1_a + alpha1_a_t) - 255 : (b1_a + alpha1_a_t);
			c4_t = a2i2_i ^ c1;
			i2a3_i = c4_t;
			c4_a_t = i2a3_a;
		end
		25 :begin
			a2i0_a = (c4_a + 255 - a1_a) >= 255 ? (c4_a + 255 - a1_a) - 255: (c4_a + 255 - a1_a);
			alpha2_a_t = a2i0_a;
			alpha2_t = a2i0_i;
			a2i1_a = (alpha1_a + 0) >= 255 ? (alpha1_a + 0) - 255 :(alpha1_a + 0);
			temp_sum_0 = alpha2_t ^ a2i1_i ^ x_s_array_0;
			a2i2_a = (alpha1_a + 1) >= 255 ? (alpha1_a + 1) - 255 :(alpha1_a + 1);	
			temp_sum_1 = alpha2_t ^ a2i2_i ^ x_s_array_1;
			a2i3_a = (alpha1_a + 2) >= 255 ? (alpha1_a + 2) - 255 :(alpha1_a + 2);	
			temp_sum_2 = alpha2_t ^ a2i3_i ^ x_s_array_2;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;

			if (!findfirstai0_t &&temp_sum_0 == 0 && error_occur) begin
				ai1_a_t = 0;
				findfirstai0_t = 1;
			end
			if (!findfirstai0_t && temp_sum_1 == 0 && error_occur) begin
				ai1_a_t = 1;
				findfirstai0_t = 1;
			end
			else if (findfirstai0 && temp_sum_1 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 1;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_2 == 0 && error_occur) begin
				ai1_a_t = 2;
				findfirstai0_t = 1;
			end
			else if (findfirstai0 && temp_sum_2 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 2;
				findfirstai1_t = 1;
			end			

		end
		26 :begin
			a2i0_a = (alpha1_a + 3) >= 255 ? (alpha1_a + 3) - 255 :(alpha1_a + 3);	
			a2i1_a = (alpha1_a + 4) >= 255 ? (alpha1_a + 4) - 255 :(alpha1_a + 4);	
			a2i2_a = (alpha1_a + 5) >= 255 ? (alpha1_a + 5) - 255 :(alpha1_a + 5);	
			a2i3_a = (alpha1_a + 6) >= 255 ? (alpha1_a + 6) - 255 :(alpha1_a + 6);	
			temp_sum_3 = alpha2 ^ a2i0_i ^ x_s_array_3;
			temp_sum_4 = alpha2 ^ a2i1_i ^ x_s_array_4;
			temp_sum_5 = alpha2 ^ a2i2_i ^ x_s_array_5;
			temp_sum_6 = alpha2 ^ a2i3_i ^ x_s_array_6;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;

			if (!findfirstai0_t &&temp_sum_3 == 0 && error_occur) begin
				ai1_a_t = 3;
				findfirstai0_t = 1;
			end
			else if (findfirstai0 && temp_sum_3 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 3;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_4== 0 && error_occur) begin
				ai1_a_t = 4;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_4 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 4;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_5 == 0 && error_occur) begin
				ai1_a_t = 5;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_5 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 5;
				findfirstai1_t = 1;
			end
			if (!findfirstai0_t && temp_sum_6 == 0 && error_occur) begin
				ai1_a_t = 6;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_6 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 6;
				findfirstai1_t = 1;
			end	
		end
		27 :begin
			a2i0_a = (alpha1_a + 7) >= 255 ? (alpha1_a + 7) - 255 :(alpha1_a + 7);	
			a2i1_a = (alpha1_a + 8) >= 255 ? (alpha1_a + 8) - 255 :(alpha1_a + 8);	
			a2i2_a = (alpha1_a + 9) >= 255 ? (alpha1_a + 9) - 255 :(alpha1_a + 9);	
			a2i3_a = (alpha1_a + 10) >= 255 ? (alpha1_a + 10) - 255 :(alpha1_a + 10);	
			temp_sum_7 = alpha2 ^ a2i0_i ^ x_s_array_7;
			temp_sum_8 = alpha2 ^ a2i1_i ^ x_s_array_8;
			temp_sum_9 = alpha2 ^ a2i2_i ^ x_s_array_9;
			temp_sum_10 = alpha2 ^ a2i3_i ^ x_s_array_10;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;

			if (!findfirstai0_t &&temp_sum_7 == 0 && error_occur) begin
				ai1_a_t = 7;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_7 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 7;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_8== 0 && error_occur) begin
				ai1_a_t = 8;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_8 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 8;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_9 == 0 && error_occur) begin
				ai1_a_t = 9;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_9 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 9;
				findfirstai1_t = 1;
			end
			if (!findfirstai0_t && temp_sum_10 == 0 && error_occur) begin
				ai1_a_t = 10;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_10 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 10;
				findfirstai1_t = 1;
			end	

		end
		28 :begin
			a2i0_a = (alpha1_a + 11) >= 255 ? (alpha1_a + 11) - 255 :(alpha1_a + 11);	
			a2i1_a = (alpha1_a + 12) >= 255 ? (alpha1_a + 12) - 255 :(alpha1_a + 12);	
			a2i2_a = (alpha1_a + 13) >= 255 ? (alpha1_a + 13) - 255 :(alpha1_a + 13);	
			a2i3_a = (alpha1_a + 14) >= 255 ? (alpha1_a + 14) - 255 :(alpha1_a + 14);	
			temp_sum_11 = alpha2 ^ a2i0_i ^ x_s_array_11;
			temp_sum_12 = alpha2 ^ a2i1_i ^ x_s_array_12;
			temp_sum_13 = alpha2 ^ a2i2_i ^ x_s_array_13;
			temp_sum_14 = alpha2 ^ a2i3_i ^ x_s_array_14;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			if (!findfirstai0_t &&temp_sum_11 == 0 && error_occur) begin
				ai1_a_t = 11;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_11 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 11;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_12== 0 && error_occur) begin
				ai1_a_t = 12;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_12 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 12;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_13 == 0 && error_occur) begin
				ai1_a_t = 13;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_13 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 13;
				findfirstai1_t = 1;
			end
			if (!findfirstai0_t && temp_sum_14 == 0 && error_occur) begin
				ai1_a_t = 14;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_14 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 14;
				findfirstai1_t = 1;
			end	
		end
		29 :begin
			a2i0_a = (alpha1_a + 15) >= 255 ? (alpha1_a + 15) - 255 :(alpha1_a + 15);	
			a2i1_a = (alpha1_a + 16) >= 255 ? (alpha1_a + 16) - 255 :(alpha1_a + 16);	
			a2i2_a = (alpha1_a + 17) >= 255 ? (alpha1_a + 17) - 255 :(alpha1_a + 17);	
			a2i3_a = (alpha1_a + 18) >= 255 ? (alpha1_a + 18) - 255 :(alpha1_a + 18);	
			temp_sum_15= alpha2 ^ a2i0_i ^ x_s_array_15;
			temp_sum_16 = alpha2 ^ a2i1_i ^ x_s_array_16;
			temp_sum_17 = alpha2 ^ a2i2_i ^ x_s_array_17;
			temp_sum_18 = alpha2 ^ a2i3_i ^ x_s_array_18;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;
			if (!findfirstai0_t &&temp_sum_15 == 0 && error_occur) begin
				ai1_a_t = 15;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_15 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 15;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_16== 0 && error_occur) begin
				ai1_a_t = 16;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_16 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 16;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_17 == 0 && error_occur) begin
				ai1_a_t = 17;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_17 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 17;
				findfirstai1_t = 1;
			end
			if (!findfirstai0_t && temp_sum_18 == 0 && error_occur) begin
				ai1_a_t = 18;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_18 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 18;
				findfirstai1_t = 1;
			end	
		end
		30 :begin
			a2i0_a = (alpha1_a + 19) >= 255 ? (alpha1_a + 19) - 255 :(alpha1_a + 19);	
			a2i1_a = (alpha1_a + 20) >= 255 ? (alpha1_a + 20) - 255 :(alpha1_a + 20);	
			a2i2_a = (alpha1_a + 21) >= 255 ? (alpha1_a + 21) - 255 :(alpha1_a + 21);	
			a2i3_a = (alpha1_a + 22) >= 255 ? (alpha1_a + 22) - 255 :(alpha1_a + 22);	
			temp_sum_19 = alpha2 ^ a2i0_i ^ x_s_array_19;
			temp_sum_20 = alpha2 ^ a2i1_i ^ x_s_array_20;
			temp_sum_21 = alpha2 ^ a2i2_i ^ x_s_array_21;
			temp_sum_22 = alpha2 ^ a2i3_i ^ x_s_array_22;
			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1;
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;
			if (!findfirstai0_t &&temp_sum_19 == 0 && error_occur) begin
				ai1_a_t = 19;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_19 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 19;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_20 == 0 && error_occur) begin
				ai1_a_t = 20;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_20 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 21;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_21 == 0 && error_occur) begin
				ai1_a_t = 21;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_21 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 21;
				findfirstai1_t = 1;
			end
			if (!findfirstai0_t && temp_sum_22 == 0 && error_occur) begin
				ai1_a_t = 22;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_22 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 22;
				findfirstai1_t = 1;
			end	
		end
		31 :begin
			a2i0_a = (alpha1_a + 23) >= 255 ? (alpha1_a + 23) - 255 :(alpha1_a + 23);	
			a2i1_a = (alpha1_a + 24) >= 255 ? (alpha1_a + 24) - 255 :(alpha1_a + 24);	
			a2i2_a = (alpha1_a + 25) >= 255 ? (alpha1_a + 25) - 255 :(alpha1_a + 25);	
			temp_sum_23 = alpha2 ^ a2i0_i ^ x_s_array_23;
			temp_sum_24 = alpha2 ^ a2i1_i ^ x_s_array_24;
			temp_sum_25 = alpha2 ^ a2i2_i ^ x_s_array_25;

			findfirstai0_t = findfirstai0; 
			findfirstai1_t = findfirstai1; 
			ai1_a_t = ai1_a;
			ai2_a_t = ai2_a;
			if (!findfirstai0_t && temp_sum_23 == 0 && error_occur) begin
				ai1_a_t = 23;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_23 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 23;
				findfirstai1_t = 1;
			end

			if (!findfirstai0_t && temp_sum_24 == 0 && error_occur) begin
				ai1_a_t = 24;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_24 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 24;
				findfirstai1_t = 1;
			end			
			if (!findfirstai0_t && temp_sum_25 == 0 && error_occur) begin
				ai1_a_t = 25;
				findfirstai0_t = 1;
			end
			else if (findfirstai0_t && temp_sum_25 == 0 && error_occur && !findfirstai1_t) begin
				ai2_a_t = 25;
				findfirstai1_t = 1;
			end


		end
		32 :begin
			a2i0_a = ai1_a;			
			a2i1_a = ai2_a;			
			a2i2_a = ai1_a << 1;			
			a2i3_a = ai2_a << 1;
			ai1_t = a2i0_i;			
			ai2_t = a2i1_i; 			
			ai1_s_t = a2i2_i; 			
			ai2_s_t = a2i3_i;	
		end
		33 : begin
			if (findfirstai1) begin
				a2i0_a = (b2_a + diff_a) >= 255?(b2_a + diff_a) - 255:(b2_a + diff_a);
				b3_t = b1 ^ a2i0_i;
				i2a0_i = b3_t;
				b3_a_t = i2a0_a;
				a2i1_a = (c2_a + diff_a) >= 255?(c2_a + diff_a) -255:(c2_a + diff_a);
				c3_t = c1 ^ a2i1_i;
				i2a1_i = c3_t;
				c3_a_t = i2a1_a;
				Y2_a_t = (c3_a_t + 255 - b3_a_t) >= 255? c3_a_t + 255 - b3_a_t - 255:(c3_a_t + 255 - b3_a_t);
				a2i2_a =  (b1_a + Y2_a_t) >= 255 ? b1_a + Y2_a_t - 255 : (b1_a + Y2_a_t);
				c4_t = a2i2_i ^ c1;
				i2a2_i = c4_t;
				c4_a_t = i2a2_a;
				Y1_a_t = (c4_a_t + 255 - a1_a) >= 255 ? (c4_a_t - a1_a): (c4_a_t + 255 - a1_a);
 			end
			else begin
				c3_a_t =ai1_a;
				b3_t = S0;
				i2a0_i = b3_t;
				b3_a_t = i2a0_a;
				Y1_a_t = (b3_a_t + (255 - c3_a_t)) >= 255? (b3_a_t - c3_a_t) : (b3_a_t + (255 - c3_a_t));
				a2i0_a = (Y1_a_t + ai1_a)>=255 ? (Y1_a_t + ai1_a)-255:(Y1_a_t + ai1_a);
				offset1_t = a2i0_i;
				// $display("c3_a = %d, b3_a_t = %d, Y1_a_t=%d", c3_a, b3_a_t, Y1_a_t);
			end
		end
		34 : begin
			a2i0_a = (Y1_a + ai1_a)>= 255 ? Y1_a + ai1_a - 255: (Y1_a + ai1_a);
			a2i1_a = (Y2_a + ai2_a)>=255?(Y2_a + ai2_a)-255:(Y2_a + ai2_a);
			offset1_t = a2i0_i;
			offset2_t = a2i1_i;
		end
		35 : begin
			// $display("offset1 = %d offset2 = %d", offset1, offset2);		
		end
		36 : begin
			
		end
	endcase

end
always @(posedge clk) begin
	if(!srstn) begin
		for(i=0; i<26; i=i+1)
			alpha_array[i] <= 0;
		findfirstai0 <= 0;
		findfirstai1 <= 0;
		decoding_finish <= 0;
		ai1_a <= 27;
		ai2_a <= 27;
	end
	else begin
		case(err_timer )
		0: begin
			alpha_array[0] <= i2a0_a;
			alpha_array[1] <= i2a1_a;
			alpha_array[2] <= i2a2_a;
			alpha_array[3] <= i2a3_a;
			S0 <= i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
		end
		1: begin
			alpha_array[4] <= i2a0_a;
			alpha_array[5] <= i2a1_a;
			alpha_array[6] <= i2a2_a;
			alpha_array[7] <= i2a3_a;
			// S0 4~7
			S0 <= S0 ^ i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
			// S1 0~3
			S1 <= S1_t;

		end
		2: begin
			alpha_array[8] <= i2a0_a;
			alpha_array[9] <= i2a1_a;
			alpha_array[10] <= i2a2_a;
			alpha_array[11] <= i2a3_a;		
			// S0 8~11
			S0 <= S0 ^ i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
			// S1 4~7
			S1 <= S1_t;
		end
		3: begin
			alpha_array[12] <= i2a0_a;
			alpha_array[13] <= i2a1_a;
			alpha_array[14] <= i2a2_a;
			alpha_array[15] <= i2a3_a;						
			// S0 12~15
			S0 <= S0 ^ i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
			// S1 8~11
			S1 <= S1_t;
		end
		4: begin
			alpha_array[16] <= i2a0_a;
			alpha_array[17] <= i2a1_a;
			alpha_array[18] <= i2a2_a;
			alpha_array[19] <= i2a3_a;						
			// S0 16~19
			S0 <= S0 ^ i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
			// S1 12~15
			S1 <= S1_t;
		end
		5: begin
			alpha_array[20] <= i2a0_a;
			alpha_array[21] <= i2a1_a;
			alpha_array[22] <= i2a2_a;
			alpha_array[23] <= i2a3_a;
			// S0 20~23			
			S0 <= S0 ^ i2a0_i ^ i2a1_i ^ i2a2_i ^ i2a3_i;
			// S1 16~19
			S1 <= S1_t;
			// $display("t= 5, S1 = %d, S0 =%d", S1, S0);
		end
		6: begin
			alpha_array[24] <= i2a0_a;
			alpha_array[25] <= i2a1_a;
			// S0 24~25
			S0 <= S0 ^ i2a0_i ^ i2a1_i;
			// S1 20~23
			S1 <= S1_t;
		end
		7: begin
			// S1 24~25
			S1 <= S1_t;
		end
		8: begin
			S2 <= S2_t;

		end
		9: begin

			S2 <= S2_t;

		end
		10: begin

			S2 <= S2_t;

		end
		11: begin

			S2 <= S2_t;
		end
		12: begin

			S2 <= S2_t;
		end
		13: begin

			S2 <= S2_t;
		end
		14 :begin

			S2 <= S2_t;
		end       
		15 :begin
			S3 <= S3_t;
		end       
		16 :begin

			S3 <= S3_t;
		end       
		17 :begin

			S3 <= S3_t;
		end       
		18 :begin
			S3 <= S3_t;
		end       
		19 :begin
			S3 <= S3_t;
		end
		20 :begin
			S3 <= S3_t;
			S0_a <= i2a0_a;
			S1_a <= i2a1_a;
			S2_a <= i2a2_a;
		end
		21 :begin
			S3_a <= i2a0_a;
			S3 <= S3_t;
		end
		22 :begin
			// $display("t = 22, S0_a = %d, S1_a = %d, S2_a = %d, S3_a=%d", S0_a, S1_a, S2_a, S3_a);
			// $display("t= 22,S3_a = %d", S3_a);
			if (S0_a >= S1_a) begin
			    a1 <= S0;
			    b1 <= S1;
			    c1 <= S2;
			    a2 <= S1;
			    b2 <= S2;
			    c2 <= S3;
			end
			else begin
			    a2 <= S0;
			    b2 <= S1;
			    c2 <= S2;
			    a1 <= S1;
			    b1 <= S2;
			    c1 <= S3;
			end
		end

		23 :begin
			a1_a <=	i2a0_a;
			a2_a <=	i2a1_a;
			b1_a <= i2a2_a;
			b2_a <= i2a3_a;
			b3 <= b3_t;
			// $display("t = 23, b3_t= %d", b3_t);
			// $display("t = 23, a2i0_i= %d, b1 ^ a2i0_i = %d", a2i0_i, b1 ^ a2i0_i);
		end
		24 :begin
			// $display("t = 24, b3 = %d, b2_a = %d, b1_a = %d, c1 = %d", b3, b2_a, b1_a, c1);
			b3_a <= b3_a_t;
			c2_a <= c2_a_t; 
			c3 <= c3_t;
			c3_a <= c3_a_t;
			alpha1_a <= alpha1_a_t;
			alpha1 <= alpha1_t;
			c4 <= c4_t;
			c4_a <= c4_a_t;
		end
		25 :begin
			// $display("t = 25, a1_a = %d,\n a2_a = %d, b1_a = %d\n, b2_a = %d,\n alpha1_a = %d\n b3_a = %d, c2_a = %d, c3_a = %d,\n c4_a = %d",
			//  a1_a, a2_a, b1_a, b2_a, alpha1_a, b3_a, c2_a, c3_a, c4_a);
			// $display("-------------------------t = 25 c3_a = %d", c3_a);
			// $display("-------------------------t = 25 b3_a = %d", b3_a);
			// $display("-------------------------t = 25 c4_a = %d", c4_a);
			// $display("-------------------------t = 25 alpha1_a = %d", alpha1_a);
			alpha2 <= alpha2_t;
			alpha2_a <= alpha2_a_t;
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;

		end
		26 :begin
			// $display("-------------------------t =26 alpha2_a = %d", alpha2_a);
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;
		end
		27 :begin
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;

		end
		28 :begin
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;
		end
		29 :begin
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;
		end
		30 :begin
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;
		end
		31 :begin
			findfirstai0 <= findfirstai0_t;
			findfirstai1 <= findfirstai1_t;
			ai1_a <= ai1_a_t;
			ai2_a <= ai2_a_t;
		end
		32 :begin
			// $display("t = 32 ai1_a = %d ai2_a = %d", ai1_a, ai2_a);
			ai1 <= ai1_t; 			
			ai2 <= ai2_t; 			
			ai1_s <= ai1_s_t; 			
			ai2_s <= ai2_s_t ;
			if (findfirstai1) begin
				a1 <= ai1_s_t;
			    b1 <= ai2_s_t;
			    c1 <= S1;
			    a2 <= ai1_a;
			    b2_a <= ai2_a;
			    c2 <= S0;
			    a2_a <= ai1_a;
			    a1_a <= ai1_a << 1;
			    b1_a <= ai2_a << 1;
			    c1_a <= S1_a;
			    c2_a <= S0_a;
			end
			else begin
				b3_a <= ai1_a;
				b3 <= ai1_t;
				c3_a <= c3_a_t;
			end
		end
		33 :begin
			// $display("t=33 a1_a = %d, a2_a = %d, b1_a = %d, b2_a = %d, c1_a = %d, c2_a=%d",
//				a1_a, a2_a, b1_a, b2_a, c1_a, c2_a);
			Y2_a <= Y2_a_t;
			Y1_a <= Y1_a_t;
			offset1 <= offset1_t;
			if (!findfirstai1)
				decoding_finish <= 1;
			else
				decoding_finish <= 0;
		end
		34 :begin
			if (findfirstai1_t) begin
				offset1 <= offset1_t;
				offset2 <= offset2_t;
			end
			// $display("~~t = 34 Y1_a = %d, Y2_a=%d", Y1_a, Y2_a);
			// $display("~~t offset1 = %d, offset2=%d", offset1_t, offset2_t);
			decoding_finish <= 1;

		end
		35 :begin


		end
		36 :begin
		end
		37 :begin

		end
		38 :begin
		end
		39 :begin
		end
		40 :begin
		end
		41 :begin
		end
		42 :begin
		end
		43 :begin
		end
		44 :begin
		end


		endcase
	end
end

reg [4:0] error_position0;
reg [4:0] error_position1;

always @* begin
	error_position0 = 25 - ai1_a;
	error_position1 = 25 - ai2_a;
	for(i=0; i<26*8; i=i+1) begin
		correct_codewords[i] = codewords[i];
	end
	if (findfirstai0) begin
		correct_codewords[error_position0*8+:8] = codewords[error_position0*8+:8] ^ offset1;
	end
	if (findfirstai1) begin
		correct_codewords[error_position1*8+:8] = codewords[error_position1*8+:8] ^ offset2;
	end
end



assign x_s_array_0 = 1;
assign x_s_array_1 = 4;
assign x_s_array_2 = 16;
assign x_s_array_3 = 64;
assign x_s_array_4 = 29;
assign x_s_array_5 = 116;
assign x_s_array_6 = 205;
assign x_s_array_7 = 19;
assign x_s_array_8 = 76;
assign x_s_array_9 = 45;
assign x_s_array_10 = 180;
assign x_s_array_11 = 234;
assign x_s_array_12 = 143;
assign x_s_array_13 = 6;
assign x_s_array_14 = 24;
assign x_s_array_15 = 96;
assign x_s_array_16 = 157;
assign x_s_array_17 = 78;
assign x_s_array_18 = 37;
assign x_s_array_19 = 148;
assign x_s_array_20 = 106;
assign x_s_array_21 = 181;
assign x_s_array_22 = 238;
assign x_s_array_23 = 159;
assign x_s_array_24 = 70;
assign x_s_array_25 = 5;

endmodule



