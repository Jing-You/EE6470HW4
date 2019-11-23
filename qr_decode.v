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

parameter IMG_LEN = 64, QR_LEN = 21;
parameter IDLE = 0, READ = 1, PROCESSING = 2,  ROTATE = 3, DECODING = 4, DEMASK=5, WRITE =6, FINISH = 7, READ_AFTER=8, SEARCH = 9, CORRECING=10;
parameter topleft = 0, bottomright = 1;
parameter MAIND = 0, OFFD= 1 , TOP= 2, LEFT = 3, TOP2 = 4, CHKR=5,CHKB=6, SEARCH_FINISH=7; 
reg qr_img_temp[QR_LEN-1:0][QR_LEN-1:0];
reg qr_img[QR_LEN-1:0][QR_LEN-1:0];
wire corner[2:0][2:0];

reg [6:0] top_most_y, left_most_x;
reg [6:0] confirm_top_most_y, confirm_left_most_x;
reg [1:0] need_rotate_times;
reg [6:0] img_x;
reg [6:0] img_y;
reg corner_detect0;
reg corner_detect1;
reg corner_detect2;
reg corner_detect3;
reg [6:0] err_cnt;
reg [4:0] output_cnt;
reg [7:0] text_length;
reg searched_position;


assign corner[0][0] = 1;
assign corner[0][1] = 1;
assign corner[0][2] = 1;
assign corner[1][0] = 1;
assign corner[1][1] = 0;
assign corner[1][2] = 0;
assign corner[2][0] = 1;
assign corner[2][1] = 0;
assign corner[2][2] = 1;
reg findfirstai0, findfirstai1, findfirstai0_t, findfirstai1_t;
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

reg [8:0]S0, S0_t;
reg [8:0]S1, S1_t;
reg [8:0]S2, S2_t;
reg [8:0]S3;
reg [8:0]S4;
reg [8:0]S5;
wire error_occur = S0 != 0 || S1 != 0 || S2 != 0 || S3 != 0;

reg [7:0] codeword [25:0];
reg [8:0] alpha_array[25:0];
reg [8:0] Y1, Y2, Y1_a, Y2_a;
reg [2:0] search_state;
reg [9:0] search_cnt;

wire [7:0] a2i0_i, a2i1_i, a2i2_i, a2i3_i;
wire [7:0] i2a0_a, i2a1_a;
reg [7:0] a2i0_a, a2i1_a, a2i2_a, a2i3_a;
reg [7:0] i2a0_i, i2a1_i;

reg [7:0] img_x_buf;
reg [7:0] img_y_buf;

always @(posedge clk) begin
	if (state == WRITE || state == SEARCH_FINISH)
		img[img_y][img_x] <= sram_rdata;
end

a2i a2i0(
	.clk(clk),
	.a(a2i0_a),
	.i(a2i0_i)
);

a2i a2i1(
	.clk(clk),
	.a(a2i1_a),
	.i(a2i1_i)
);

i2a i2a0(
	.clk(clk),
	.i(i2a0_i),
	.a(i2a0_a)
);


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
		if (err_cnt>= 58 && !error_occur)begin
			state <= WRITE;
		end
		else if (findfirstai1 && findfirstai0 && err_cnt == 78) begin	
			state <= WRITE;
		end
		else if (!findfirstai1 && err_cnt == 73) begin
			state <= WRITE;
		end 
		else
			state <= DECODING;

	end
	else if (state == WRITE) begin
		
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
				// $display("search fail");
				// $finish();
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

reg decode_valid_buf;
reg [7:0] decode_jis8_code_buf;
// TODO
always @* begin
	decode_valid_buf = state == WRITE && output_cnt < text_length;
	case(output_cnt) // synopsys full_case parallel_case
	    0:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(0+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(0+1), 3'b0} 	  +:4];
	    end

	    1:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(1+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(1+1), 3'b0} 	  +:4];
	    end
	    

	    2:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(2+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(2+1), 3'b0} 	  +:4];
	    end
	    3:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(3+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(3+1), 3'b0} 	  +:4];
	    end
	    4:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(4+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(4+1), 3'b0} 	  +:4];
	    end
	    5:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(5+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(5+1), 3'b0} 	  +:4];
	    end
	    6:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(6+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(6+1), 3'b0} 	  +:4];
	    end
	    

	    7:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(7+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(7+1), 3'b0} 	  +:4];
	    end
	    

	    8:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(8+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(8+1), 3'b0} 	  +:4];
	    end
	    

	    9:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(9+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(9+1), 3'b0} 	  +:4];
	    end
	    

	    10:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(10+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(10+1), 3'b0} 	  +:4];
	    end
	    

	    11:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(11+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(11+1), 3'b0} 	  +:4];
	    end
	    

	    12:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(12+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(12+1), 3'b0} 	  +:4];
	    end
	    

	    13:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(13+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(13+1), 3'b0} 	  +:4];
	    end
	    

	    14:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(14+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(14+1), 3'b0} 	  +:4];
	    end
	    

	    15:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(15+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(15+1), 3'b0} 	  +:4];
	    end
	    

	    16:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(16+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(16+1), 3'b0} 	  +:4];
	    end
	    

	    17:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(17+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(17+1), 3'b0} 	  +:4];
	    end
	    

	    18:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(18+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(18+1), 3'b0} 	  +:4];
	    end
	    

	    19:begin
	        decode_jis8_code_buf[3:0] = correct_codewords[{(19+2), 3'b0}  +4  +:4];
	        decode_jis8_code_buf[7:4] = correct_codewords[{(19+1), 3'b0} 	  +:4];
	    end

    endcase
end

always @(posedge clk) begin
	decode_valid <= decode_valid_buf;
	decode_jis8_code <= decode_jis8_code_buf;
end


reg [8:0] sdiff_a;
reg [8:0] alpha1, alpha2;
reg [8:0] alpha1_t, alpha2_t;
reg [8:0] alpha1_a, alpha2_a;
reg [8:0] alpha1_a_t, alpha2_a_t;
reg [8:0] S0_a, S1_a, S2_a, S3_a, S4_a, S5_a;
reg [8:0] S3_t;
reg [8:0] S0_a_t, S1_a_t, S2_a_t, S3_a_t;
reg [8:0] S6, S7, S6_a, S7_a, S8, S8_a;
reg [8:0] S9, S9_a;
reg [8:0] S10, S10_a;
reg [8:0] ai1, ai2;
reg [8:0] ai1_a, ai2_a;
reg [8:0] temp_sum;
reg [8:0] a1, a2, b1, b2, c3, c4;
reg [8:0] b3_t, b3_a_t;
reg [8:0] c3_t, c4_t;
reg [8:0] c3_a_t, c4_a_t;
reg [8:0] c1, c2, c1_a, c2_a;
reg [8:0] b3, b3_a;
reg [8:0] a1_a, a2_a, b1_a, b2_a, c3_a, c4_a;
reg [8:0] ai1_s, ai2_s;
reg [8:0] ai1_s_a, ai2_s_a;
reg [8:0] diff_a;
reg [8:0] unknown_x, unknown_y;
reg [8:0] unknown_x_a, unknown_y_a;
reg [7:0] offset1, offset2;
reg [8:0] correcting_cnt;
reg [8:0] alpha1x;
wire [7:0] x_s_array[25:0];
reg [8:0]  ai1_a_t, ai2_a_t;
reg [8:0] ai1_t, ai2_t;
reg [8:0] ai1_s_t, ai2_s_t;
reg [8:0] Y1_a_t, Y2_a_t;
reg [8:0] Y1_t, Y2_t;
reg [7:0] offset2_t, offset1_t;
always @(posedge clk) begin
	if (!srstn) begin
		correcting_cnt <= 0;
	end
	else if (state == CORRECING) begin
		correcting_cnt <= correcting_cnt + 1; 
	end
	else begin
		correcting_cnt <= 0;
	end
end
reg [8:0] temp_exp;
reg [2:0] decoding_state;


always @(posedge clk) begin
	if(!srstn) begin
		err_cnt <= 0;		
	end
	else if (state == DECODING)
		err_cnt <= err_cnt + 1;
	else begin
		err_cnt <= err_cnt;
	end
end
// S_FF
always @(posedge clk) begin
	if (!srstn) begin
		// reset
		for(i=0; i<26; i=i+1)
			alpha_array[i] <= 0;
		S0 <= 0;
		S1 <= 0;
		S2 <= 0;
		S3 <= 0;
		findfirstai0 <= 0;
		findfirstai1 <= 0;
	end
	else if (state == DECODING)begin
		for(i=0; i<26; i=i+1) begin
			if (err_cnt == i+1)
				alpha_array[i] <= i2a0_a;
		end
		if (decoding_state == 0) begin
			if (err_cnt < 26) begin
				S0 <= S0_t;			
			end
			if ( 2 < err_cnt && err_cnt < 29) begin
				S1 <= S1_t;
				S2 <= S2_t;
			end
			if (30 <= err_cnt && err_cnt < 30+26) begin
				S3 <= S3_t;			
			end
			if (err_cnt == 30)
				S0_a <= i2a0_a; 
			if (err_cnt == 31)
				S1_a <= i2a0_a; 
			if (err_cnt == 32)
				S2_a <= i2a0_a; 
			if (err_cnt == 57)
				S3_a <= i2a0_a;
		end
		else if (decoding_state == 1) begin
			if (err_cnt == 59)
				b3 <= b3_t;
			if (err_cnt == 60) begin
				b3_a <= b3_a_t;
			end
			if (err_cnt == 60)
				c3 <= c3_t;
			if (err_cnt == 61)
				c3_a <= c3_a_t;
			if (err_cnt == 61)
				alpha1_a <= alpha1_a_t;
			if (err_cnt == 63)
				c4 <= c4_t;
			if (err_cnt == 64)
				c4_a <= c4_a_t;
			if (err_cnt == 64)
				alpha1 <= alpha1_t;
			if (err_cnt == 65)
				alpha2 <= alpha2_t;
		end
		else if (decoding_state == 2) begin
			if (err_cnt == 67) begin
				ai1_a <= ai1_a_t;
			 	ai2_a <= ai2_a_t;
				findfirstai0 <= findfirstai0_t;
				findfirstai1 <= findfirstai1_t;						
			end
		end
		else begin
			if (err_cnt == 69) begin
				ai1 <= ai1_t;
				ai2 <= ai2_t;
			end
			if (err_cnt == 70) begin
				ai1_s <= ai1_s_t; 
				ai2_s <= ai2_s_t; 
			end	
			if (findfirstai1) begin
				if (err_cnt == 72) begin
					b3 <= b3_t;
				end
				if (err_cnt == 72) begin
					c3 <= c3_t;
				end
				if (err_cnt == 73)
					b3_a <= b3_a_t;
				if (err_cnt == 74) begin
					c3_a <= c3_a_t;
					Y2_a <= Y2_a_t;			
				end
				if (err_cnt == 75)
					Y2 <= Y2_t;
				if (err_cnt == 76)
					c4 <= c4_t;
				if (err_cnt == 77)
					Y1_a <= Y1_a_t;
				if (err_cnt == 78) begin
					offset1 <= offset1_t;
					offset2 <= offset2_t;
				end
			end
			else begin
				if (err_cnt == 72)
					offset1 <= offset1_t;
			end
		end
		
	end
end


always @* begin
	decoding_state = 0;
	if (err_cnt < 58)
		decoding_state = 0;
	else if (err_cnt >= 58 && err_cnt < 66)
		decoding_state = 1;
	else if (err_cnt >= 66 && err_cnt <68)
		decoding_state = 2;
	else if (err_cnt >= 68 )
		decoding_state = 3;

end

// TODO 優化alpha_array selection
always @* begin	    
	S0_t = 0;
	S1_t = 0;
	S2_t = 0;
	S3_t = 0;
	alpha1_a_t = 0;
	alpha2_a_t = 0;
	alpha1_t = 0;
	alpha2_t = 0;
	ai1_a_t = 0;
	ai2_a_t = 0;
	findfirstai0_t = 0;
	findfirstai1_t = 0;
	ai1_t = 0;
	ai2_t = 0;
	ai1_s_t = 0;
	ai2_s_t = 0;
	b3_t = 0;
	b3_a_t = 0;
	c3_t = 0;
	c3_a_t = 0;
	c4_t = 0;
	c4_a_t = 0;
	Y1_a_t = 0;
	offset1_t = 0;
	offset2_t = 0;
	Y2_a_t = 0;
	Y2_t = 0;
	a2i0_a = 0;
	a2i1_a = 0;
	i2a0_i = 0;

	case (decoding_state) // synopsys full_case parallel_case
		0: begin
			//  compute alpha_array
			for (i=0; i<26; i=i+1)
				if (err_cnt == i)
		        	i2a0_i = codewords[i*8 +:8];

			if ( err_cnt < 26)
				S0_t = S0 ^ codewords[(err_cnt)*8 +:8];

			for (i=0; i<26; i=i+1) begin
				if (err_cnt==i+2) 
					a2i0_a = (alpha_array[i] + (25-i)*1) >= 255 ?(alpha_array[i] + (25-i)*1) - 255: (alpha_array[i] + (25-i)*1);
				if (err_cnt==i+2) 
					a2i1_a = (alpha_array[i] + (25-i)*2) >= 255 ?(alpha_array[i] + (25-i)*2) - 255: (alpha_array[i] + (25-i)*2);
				if (err_cnt==i+3) begin
					S1_t = S1 ^ a2i0_i;
					S2_t = S2 ^ a2i1_i;
				end
			end
			if ( 2 < err_cnt && err_cnt < 29) begin
					S1_t = S1 ^ a2i0_i;
					S2_t = S2 ^ a2i1_i;
			end

			for (i=0; i<26; i=i+1) begin
				if (err_cnt==i+3+26) 
					a2i1_a = (alpha_array[i] + (25-i)*3) >= 255 ?(alpha_array[i] + (25-i)*3) - 255: (alpha_array[i] + (25-i)*3);
				if (err_cnt==i+3+26+1) 
					S3_t = S3 ^ a2i1_i;
			end


			// compute S0_a_t
		    if (err_cnt==29)
		    	i2a0_i = S0;
			// compute S1_a_t
		    if (err_cnt==30)
		    	i2a0_i = S1;
			// compute S2_a_t
		    if (err_cnt==31)
		    	i2a0_i = S2;
			// compute S3_a_t
			if (err_cnt == 56)
				i2a0_i = S3; 

		end
		1: begin
			if (S0_a > S1_a) begin
			    a1 = S0;
			    b1 = S1;
			    c1 = S2;
			    a2 = S1;
			    b2 = S2;
			    c2 = S3;
			    a1_a = S0_a;
			    b1_a = S1_a;
			    c1_a = S2_a;
			    a2_a = S1_a;
			    b2_a = S2_a;
			    c2_a = S3_a;
			end
			else begin
			    a2 = S0;
			    b2 = S1;
			    c2 = S2;
			    a1 = S1;
			    b1 = S2;
			    c1 = S3;
			    a1_a = S1_a;
			    b1_a = S2_a;
			    c1_a = S3_a;
			    a2_a = S0_a;
			    b2_a = S1_a;
			    c2_a = S2_a;
			end
			diff_a = a1_a - a2_a;

			if (err_cnt == 58) begin
				a2i0_a = (b2_a + diff_a) >= 255 ? (b2_a + diff_a) - 255 : (b2_a + diff_a);
			end

			b3_t = b1 ^ a2i0_i; // t = 59
			if (err_cnt == 59) begin
				i2a0_i = b3_t;		
			end
			b3_a_t = i2a0_a; // t = 60
			if (err_cnt == 59) begin
				a2i1_a = (c2_a + diff_a) >= 255 ? (c2_a + diff_a) - 255 : (c2_a + diff_a);
			end
			c3_t = c1 ^ a2i1_i; // t = 60
			if (err_cnt == 60) begin
				i2a0_i = c3_t;
			end
			c3_a_t = i2a0_a; // t = 61

			// t = 61
			alpha1_a_t = (c3_a_t + 255 - b3_a) >= 255 ?(c3_a_t + 255 - b3_a) - 255:(c3_a_t + 255 - b3_a);
			// t = 62
			if (err_cnt== 62) begin
				a2i0_a = (b1_a + alpha1_a) >= 255 ? (b1_a + alpha1_a) - 255 : (b1_a + alpha1_a);
			end
			// t = 63
			c4_t = a2i0_i ^ c1;

			// t = 63
			if (err_cnt == 63) begin
				i2a0_i = c4_t;
				a2i0_a = alpha1_a;
			end
			// t = 64
			alpha1_t = a2i0_i;
			// t = 64
			c4_a_t = i2a0_a;
			// t = 64
			if (err_cnt == 64) begin
				a2i1_a = (c4_a_t + 255 - a1_a) >= 255 ? (c4_a_t + 255 - a1_a) - 255: (c4_a_t + 255 - a1_a);
			end
			alpha2_t = a2i1_i;
			end
		2: begin
			ai1_a_t = 27;
			ai2_a_t = 27;
			alpha1x = alpha1;
			findfirstai0_t = 0;
			findfirstai1_t = 0;
			for (i=0; i<26; i = i+1) begin
				temp_sum = alpha2 ^ alpha1x ^ x_s_array[i];
				if (!findfirstai0_t && temp_sum == 0 && error_occur) begin
					ai1_a_t = i;
					findfirstai0_t = 1;
				end
				else if (findfirstai0_t && temp_sum == 0 && error_occur && !findfirstai1_t) begin
					ai2_a_t = i;
					findfirstai1_t = 1;				
				end
				alpha1x = {alpha1x, 1'b0};
				alpha1x = alpha1x > 255 ? alpha1x ^ 285 : alpha1x;
			end
		end
		3: begin
			// ***********************
			// t = 68
			if (err_cnt == 68) begin
				a2i0_a = ai1_a;
				a2i1_a = ai2_a;
			end
			// t = 69
			ai1_t = a2i0_i;
			ai2_t = a2i1_i;
			// t = 69
			if (err_cnt == 69) begin
				a2i0_a = {ai1_a, 1'b0};
				a2i1_a = {ai2_a, 1'b0};
			end
			// t = 70
			ai1_s_t = a2i0_i;
			ai2_s_t = a2i1_i;
			// **********************
			if (findfirstai1) begin
			    a1 = ai1_s;
			    b1 = ai2_s;
			    c1 = S1;
			    a2 = ai1;
			    b2 = ai2;
			    c2 = S0;
				a1_a = {ai1_a, 1'b0};
				a2_a = ai1_a;
				b1_a = {ai2_a, 1'b0};
				b2_a = ai2_a;
				c1_a = S1_a;
				c2_a = S0_a;
				diff_a = a1_a - a2_a;
				//////////////////////////////
				if (err_cnt == 71) begin
					a2i0_a = (b2_a + diff_a) >= 255 ? (b2_a + diff_a) - 255 : (b2_a + diff_a);
				end
				b3_t = b1 ^ a2i0_i; // t = 72
				if (err_cnt == 71) begin
					a2i1_a = (c2_a + diff_a) >= 255 ? (c2_a + diff_a) - 255 : (c2_a + diff_a);
				end
				if (err_cnt == 72)
					i2a0_i = b3_t;

				b3_a_t = i2a0_a; // t=73
				c3_t = c1 ^ a2i1_i; // t = 72
				if (err_cnt == 73) begin
					i2a0_i = c3;

				end
				c3_a_t = i2a0_a; // t = 74
				Y2_a_t = (c3_a_t + 255 - b3_a) >= 255? c3_a_t + 255 - b3_a - 255:(c3_a_t + 255 - b3_a);
				if (err_cnt == 74) begin
					i2a0_i = c3_t;
					a2i0_a = Y2_a_t;
				end
				Y2_t = a2i0_i; // t = 75
				if (err_cnt == 75) begin
					a2i0_a = (b1_a + Y2_a) >= 255 ? (b1_a + Y2_a) - 255 : (b1_a + Y2_a);
				end
				c4_t = a2i0_i ^ c1; // t = 76
				if (err_cnt == 76) begin
					i2a0_i = c4_t;
				end
				c4_a_t = i2a0_a; // t = 77
				Y1_a_t = (c4_a_t + 255 - a1_a) >= 255 ? (c4_a_t - a1_a): (c4_a_t + 255 - a1_a);
				if (err_cnt == 77) begin
					a2i0_a = (Y1_a_t + ai1_a)>= 255 ? Y1_a_t + ai1_a - 255: (Y1_a_t + ai1_a);
					a2i1_a = (Y2_a + ai2_a)>= 255?(Y2_a + ai2_a)-255:(Y2_a + ai2_a);
				end
				// t = 78
				offset1_t = a2i0_i;
				offset2_t = a2i1_i;
				
			end	
			else begin
				Y1_a_t = (S0_a + (255 - ai1_a)) >= 255? (S0_a - ai1_a) : (S0_a + (255 - ai1_a));
				if (err_cnt == 71) begin
					a2i0_a = (Y1_a_t + ai1_a)>=255 ? (Y1_a_t + ai1_a)-255:(Y1_a_t + ai1_a);
				end
				offset1_t = a2i0_i;
				offset2_t = 0;
			end
		end
	endcase

end

reg [4:0] error_position0;
reg [4:0] error_position1;

always @* begin
	error_position0 = 25 - ai1_a;
	error_position1 = 25 - ai2_a;
	for(i=0; i<26*8; i=i+1) begin
		correct_codewords[i] = codewords[i];
	end
	correct_codewords[215:208] = 0;
	if (findfirstai0) begin
		case(error_position0)
			0: correct_codewords[0*8+:8] = codewords[0*8+:8] ^ offset1; 
			1: correct_codewords[1*8+:8] = codewords[1*8+:8] ^ offset1; 
			2: correct_codewords[2*8+:8] = codewords[2*8+:8] ^ offset1;
			3: correct_codewords[3*8+:8] = codewords[3*8+:8] ^ offset1;
			4: correct_codewords[4*8+:8] = codewords[4*8+:8] ^ offset1;
			5: correct_codewords[5*8+:8] = codewords[5*8+:8] ^ offset1;
			6: correct_codewords[6*8+:8] = codewords[6*8+:8] ^ offset1;
			7: correct_codewords[7*8+:8] = codewords[7*8+:8] ^ offset1;
			8: correct_codewords[8*8+:8] = codewords[8*8+:8] ^ offset1;
			9: correct_codewords[9*8+:8] = codewords[9*8+:8] ^ offset1;
			10: correct_codewords[10*8+:8] = codewords[10*8+:8] ^ offset1;
			11: correct_codewords[11*8+:8] = codewords[11*8+:8] ^ offset1;
			12: correct_codewords[12*8+:8] = codewords[12*8+:8] ^ offset1;
			13: correct_codewords[13*8+:8] = codewords[13*8+:8] ^ offset1;
			14: correct_codewords[14*8+:8] = codewords[14*8+:8] ^ offset1;
			15: correct_codewords[15*8+:8] = codewords[15*8+:8] ^ offset1;
			16: correct_codewords[16*8+:8] = codewords[16*8+:8] ^ offset1;
			17: correct_codewords[17*8+:8] = codewords[17*8+:8] ^ offset1;
			18: correct_codewords[18*8+:8] = codewords[18*8+:8] ^ offset1;
			19: correct_codewords[19*8+:8] = codewords[19*8+:8] ^ offset1;
			20: correct_codewords[20*8+:8] = codewords[20*8+:8] ^ offset1;
			21: correct_codewords[21*8+:8] = codewords[21*8+:8] ^ offset1;
			22: correct_codewords[22*8+:8] = codewords[22*8+:8] ^ offset1;
			23: correct_codewords[23*8+:8] = codewords[23*8+:8] ^ offset1;
			24: correct_codewords[24*8+:8] = codewords[24*8+:8] ^ offset1;
			25: correct_codewords[25*8+:8] = codewords[25*8+:8] ^ offset1;
		endcase
	end
	// if (findfirstai1) begin
	// correct_codewords[error_position1*8+:8] = codewords[error_position1*8+:8] ^ offset2;
	// end


	case(error_position1)
		0: correct_codewords[0*8+:8] = codewords[0*8+:8] ^ offset2; 
		1: correct_codewords[1*8+:8] = codewords[1*8+:8] ^ offset2; 
		2: correct_codewords[2*8+:8] = codewords[2*8+:8] ^ offset2;
		3: correct_codewords[3*8+:8] = codewords[3*8+:8] ^ offset2;
		4: correct_codewords[4*8+:8] = codewords[4*8+:8] ^ offset2;
		5: correct_codewords[5*8+:8] = codewords[5*8+:8] ^ offset2;
		6: correct_codewords[6*8+:8] = codewords[6*8+:8] ^ offset2;
		7: correct_codewords[7*8+:8] = codewords[7*8+:8] ^ offset2;
		8: correct_codewords[8*8+:8] = codewords[8*8+:8] ^ offset2;
		9: correct_codewords[9*8+:8] = codewords[9*8+:8] ^ offset2;
		10: correct_codewords[10*8+:8] = codewords[10*8+:8] ^ offset2;
		11: correct_codewords[11*8+:8] = codewords[11*8+:8] ^ offset2;
		12: correct_codewords[12*8+:8] = codewords[12*8+:8] ^ offset2;
		13: correct_codewords[13*8+:8] = codewords[13*8+:8] ^ offset2;
		14: correct_codewords[14*8+:8] = codewords[14*8+:8] ^ offset2;
		15: correct_codewords[15*8+:8] = codewords[15*8+:8] ^ offset2;
		16: correct_codewords[16*8+:8] = codewords[16*8+:8] ^ offset2;
		17: correct_codewords[17*8+:8] = codewords[17*8+:8] ^ offset2;
		18: correct_codewords[18*8+:8] = codewords[18*8+:8] ^ offset2;
		19: correct_codewords[19*8+:8] = codewords[19*8+:8] ^ offset2;
		20: correct_codewords[20*8+:8] = codewords[20*8+:8] ^ offset2;
		21: correct_codewords[21*8+:8] = codewords[21*8+:8] ^ offset2;
		22: correct_codewords[22*8+:8] = codewords[22*8+:8] ^ offset2;
		23: correct_codewords[23*8+:8] = codewords[23*8+:8] ^ offset2;
		24: correct_codewords[24*8+:8] = codewords[24*8+:8] ^ offset2;
		25: correct_codewords[25*8+:8] = codewords[25*8+:8] ^ offset2;
	endcase

end

assign x_s_array[0] = 1;
assign x_s_array[1] = 4;
assign x_s_array[2] = 16;
assign x_s_array[3] = 64;
assign x_s_array[4] = 29;
assign x_s_array[5] = 116;
assign x_s_array[6] = 205;
assign x_s_array[7] = 19;
assign x_s_array[8] = 76;
assign x_s_array[9] = 45;
assign x_s_array[10] = 180;
assign x_s_array[11] = 234;
assign x_s_array[12] = 143;
assign x_s_array[13] = 6;
assign x_s_array[14] = 24;
assign x_s_array[15] = 96;
assign x_s_array[16] = 157;
assign x_s_array[17] = 78;
assign x_s_array[18] = 37;
assign x_s_array[19] = 148;
assign x_s_array[20] = 106;
assign x_s_array[21] = 181;
assign x_s_array[22] = 238;
assign x_s_array[23] = 159;
assign x_s_array[24] = 70;
assign x_s_array[25] = 5;

endmodule


