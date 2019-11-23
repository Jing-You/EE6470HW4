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

parameter IMG_LEN = 64, QR_LEN = 21;
parameter IDLE = 0, READ = 1, PROCESSING = 2,  ROTATE = 3, DECODING = 4, DEMASK=5, WRITE =6, FINISH = 7, READ_AFTER=8, SEARCH = 9, CORRECING=10;
parameter topleft = 0, bottomright = 1;
parameter MAIND = 0, OFFD= 1 , TOP= 2, LEFT = 3, TOP2 = 4, CHKR=5,CHKB=6, SEARCH_FINISH=7; 
reg qr_img_temp[QR_LEN-1:0][QR_LEN-1:0];
reg qr_img[QR_LEN-1:0][QR_LEN-1:0];
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
reg error_occur;
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

wire [7:0] a2i [255:0];
wire [7:0] i2a [255:0];

reg [8:0]S0;
reg [8:0]S1;
reg [8:0]S2;
reg [8:0]S3;
reg [8:0]S4;
reg [8:0]S5;

reg [7:0] codeword [25:0];
reg [8:0] alpha_array[25:0];
reg [8:0] Y1, Y2, Y1_a, Y2_a;
reg [2:0] search_state;
reg [9:0] search_cnt;

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
		state <= WRITE;
	end
	else if (state == WRITE) begin
		if (output_cnt == 0) begin
			// $display(" ");
			// $display("--------------------");
			// $display("S0_a= %d, S1_a =%d, S2_a =%d, S3_a= %d", S0_a, S1_a, S2_a, S3_a);
			// $display("alpha1_a = %d, alpha2_a = %d", alpha1_a, alpha2_a);
			// $display("ai1_a = %d, ai2_a = %d", ai1_a, ai2_a);
			// $display("Y1_a = %d, Y2_a = %d", Y1_a, Y2_a);
			// $display("Y1 = %d, Y2 = %d", Y1, Y2);
			// $display("--------------------");
		end
		else if (output_cnt == text_length) begin
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
	// sram_raddr = img_x + img_y * 64;
	// sram_raddr = img_x + (img_y << 6);
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

reg decode_valid_buf;
reg [7:0] decode_jis8_code_buf;

always @* begin
	decode_valid_buf = state == WRITE && output_cnt < text_length;
	// decode_jis8_code[3:0] = correct_codewords[(output_cnt+2) * 8  +4  +:4];
	decode_jis8_code_buf[3:0] = correct_codewords[{(output_cnt+2), 3'b0}  +4  +:4];
	// decode_jis8_code[7:4] = correct_codewords[(output_cnt+1) * 8	  +:4];
	decode_jis8_code_buf[7:4] = correct_codewords[{(output_cnt+1), 3'b0} 	  +:4];
end

always @(posedge clk) begin
	decode_valid <= decode_valid_buf;
	decode_jis8_code <= decode_jis8_code_buf;
end

always @* begin
	for(i=0; i<26; i=i+1) begin
		codeword[i] = codewords[i*8+:8];
	end
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
reg [8:0] diff_a;
reg [8:0] unknown_x, unknown_y;
reg [8:0] unknown_x_a, unknown_y_a;
reg [7:0] offset1, offset2;
reg [8:0]correcting_cnt;

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
always @* begin
	S0 = 0;
	S1 = 0;
	S2 = 0;
	S3 = 0;
	findfirstai0 = 0;
	findfirstai1 = 0;

	for (i=0; i<26; i=i+1) begin
		alpha_array[i] = i2a[codewords[i*8 +:8]];
	end

	for (i=0; i<26; i=i+1) begin
		S0 = S0 ^ codewords[i*8 +:8];
	end

	S0_a = i2a[S0];
	for (i=0; i<26; i=i+1) begin
		S1 = S1 ^ a2i[(alpha_array[i] + 25-i) >= 255    ?(alpha_array[i] + 25-i) - 255: (alpha_array[i] + 25-i)];
	end
	S1_a = i2a[S1];
	for (i=0; i<26; i=i+1)
		S2 = S2 ^ a2i[(alpha_array[i] + (25-i)*2) >=255 ? (alpha_array[i] + (25-i)*2) - 255 : (alpha_array[i] + (25-i)*2	)];

	for (i=0; i<26; i=i+1)
		S3 = S3 ^ a2i[(alpha_array[i] + (25-i)*3) >=255 ? (alpha_array[i] + (25-i)*3) - 255 :(alpha_array[i] + (25-i)*3	)];

	error_occur = S0 != 0 || S1 != 0 || S2 != 0 || S3 != 0;

	if (S0_a > S1_a) begin
	    a1 = S0;
	    b1 = S1;
	    c1 = S2;
	    a2 = S1;
	    b2 = S2;
	    c2 = S3;
	end
	else begin
	    a2 = S0;
	    b2 = S1;
	    c2 = S2;
	    a1 = S1;
	    b1 = S2;
	    c1 = S3;
	end
	a1_a = i2a[a1];
	a2_a = i2a[a2];
	b1_a = i2a[b1];
	diff_a = a1_a - a2_a;
	b2_a = i2a[b2];
	b3 = b1 ^ a2i[(b2_a + diff_a) >= 255 ? (b2_a + diff_a) - 255 : (b2_a + diff_a)];
	b3_a = i2a[b3];
	c2_a = i2a[c2];
	c3 = c1 ^ a2i[(c2_a + diff_a) >= 255 ? (c2_a + diff_a) - 255 : (c2_a + diff_a) ];
	c3_a = i2a[c3];
	unknown_y_a = (c3_a + 255 - b3_a) >= 255 ?(c3_a + 255 - b3_a) - 255:(c3_a + 255 - b3_a);
	unknown_y = a2i[unknown_y_a];
	c4 = a2i[(b1_a + unknown_y_a) >= 255 ? (b1_a + unknown_y_a) - 255 : (b1_a + unknown_y_a)] ^ c1;
	c4_a = i2a[c4];
	unknown_x_a = (c4_a + 255 - a1_a) >= 255 ? (c4_a + 255 - a1_a) - 255: (c4_a + 255 - a1_a);
	unknown_x = a2i[unknown_x_a];

	alpha1 = unknown_y;
	alpha2 = unknown_x;
	alpha1_a = i2a[alpha1];

	ai1_a = 27;
	ai2_a = 27;

	for (i=0; i<26; i = i+1) begin
		temp_sum = alpha2 ^ a2i[(alpha1_a + i) >= 255 ? (alpha1_a + i) - 255 :(alpha1_a + i)] ^ a2i[2 * i];
		if (!findfirstai0 && temp_sum == 0 && error_occur) begin
			ai1_a = i;
			findfirstai0 = 1;
		end
		else if (findfirstai0 && temp_sum == 0 && error_occur && !findfirstai1) begin
			ai2_a = i;
			findfirstai1 = 1;				
		end
	end
	// ***********************
	ai1 = a2i[ai1_a];
	ai2 = a2i[ai2_a];
	ai1_s = a2i[ai1_a*2];
	ai2_s = a2i[ai2_a*2];
	// **********************
	if (findfirstai1) begin
	    a1 = ai1_s;
	    b1 = ai2_s;
	    c1 = S1;
	    a2 = ai1;
	    b2 = ai2;
	    c2 = S0;
		a1_a = i2a[a1];
		a2_a = i2a[a2];
		b1_a = i2a[b1];
		diff_a = a1_a - a2_a;
		b2_a = i2a[b2];
		b3 = b1 ^ a2i[(b2_a + diff_a) >= 255?(b2_a + diff_a) - 255:(b2_a + diff_a) ];
		b3_a = i2a[b3];
		c2_a = i2a[c2];
		c3 = c1 ^ a2i[(c2_a + diff_a) >= 255?(c2_a + diff_a) -255:(c2_a + diff_a)];
		c3_a = i2a[c3];
		unknown_y_a = (c3_a + 255 - b3_a) >= 255? c3_a + 255 - b3_a - 255:(c3_a + 255 - b3_a);
		unknown_y = a2i[unknown_y_a];
		c4 = a2i[(b1_a + unknown_y_a) >= 255 ? b1_a + unknown_y_a - 255 : (b1_a + unknown_y_a)] ^ c1;
		c4_a = i2a[c4];
		unknown_x_a = (c4_a + 255 - a1_a) >= 255 ? (c4_a - a1_a): (c4_a + 255 - a1_a);
		unknown_x = a2i[unknown_x_a];
		Y2 = unknown_y;
		Y1 = unknown_x;
		Y2_a = i2a[Y2];
		Y1_a = i2a[Y1];
		offset1 = a2i[(Y1_a + ai1_a)>= 255 ? Y1_a + ai1_a - 255: (Y1_a + ai1_a)];
		offset2 = a2i[(Y2_a + ai2_a)>=255?(Y2_a + ai2_a)-255:(Y2_a + ai2_a)];
	end else begin
		c3_a = ai1_a;
		b3 = S0;
		b3_a = i2a[b3];
		unknown_y_a = (b3_a + (255 - c3_a)) >= 255? (b3_a - c3_a) : (b3_a + (255 - c3_a));
		Y1_a = unknown_y_a;
		offset1 = a2i[(Y1_a + ai1_a)>=255 ? (Y1_a + ai1_a)-255:(Y1_a + ai1_a)];
		offset2 = 0;
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
	correct_codewords[215:208] = 0;
	if (findfirstai0) begin
		correct_codewords[error_position0*8+:8] = codewords[error_position0*8+:8] ^ offset1;
	end
	if (findfirstai1) begin
		correct_codewords[error_position1*8+:8] = codewords[error_position1*8+:8] ^ offset2;
	end
end

assign a2i[0] = 1;
assign a2i[1] = 2;
assign a2i[2] = 4;
assign a2i[3] = 8;
assign a2i[4] = 16;
assign a2i[5] = 32;
assign a2i[6] = 64;
assign a2i[7] = 128;
assign a2i[8] = 29;
assign a2i[9] = 58;
assign a2i[10] = 116;
assign a2i[11] = 232;
assign a2i[12] = 205;
assign a2i[13] = 135;
assign a2i[14] = 19;
assign a2i[15] = 38;
assign a2i[16] = 76;
assign a2i[17] = 152;
assign a2i[18] = 45;
assign a2i[19] = 90;
assign a2i[20] = 180;
assign a2i[21] = 117;
assign a2i[22] = 234;
assign a2i[23] = 201;
assign a2i[24] = 143;
assign a2i[25] = 3;
assign a2i[26] = 6;
assign a2i[27] = 12;
assign a2i[28] = 24;
assign a2i[29] = 48;
assign a2i[30] = 96;
assign a2i[31] = 192;
assign a2i[32] = 157;
assign a2i[33] = 39;
assign a2i[34] = 78;
assign a2i[35] = 156;
assign a2i[36] = 37;
assign a2i[37] = 74;
assign a2i[38] = 148;
assign a2i[39] = 53;
assign a2i[40] = 106;
assign a2i[41] = 212;
assign a2i[42] = 181;
assign a2i[43] = 119;
assign a2i[44] = 238;
assign a2i[45] = 193;
assign a2i[46] = 159;
assign a2i[47] = 35;
assign a2i[48] = 70;
assign a2i[49] = 140;
assign a2i[50] = 5;
assign a2i[51] = 10;
assign a2i[52] = 20;
assign a2i[53] = 40;
assign a2i[54] = 80;
assign a2i[55] = 160;
assign a2i[56] = 93;
assign a2i[57] = 186;
assign a2i[58] = 105;
assign a2i[59] = 210;
assign a2i[60] = 185;
assign a2i[61] = 111;
assign a2i[62] = 222;
assign a2i[63] = 161;
assign a2i[64] = 95;
assign a2i[65] = 190;
assign a2i[66] = 97;
assign a2i[67] = 194;
assign a2i[68] = 153;
assign a2i[69] = 47;
assign a2i[70] = 94;
assign a2i[71] = 188;
assign a2i[72] = 101;
assign a2i[73] = 202;
assign a2i[74] = 137;
assign a2i[75] = 15;
assign a2i[76] = 30;
assign a2i[77] = 60;
assign a2i[78] = 120;
assign a2i[79] = 240;
assign a2i[80] = 253;
assign a2i[81] = 231;
assign a2i[82] = 211;
assign a2i[83] = 187;
assign a2i[84] = 107;
assign a2i[85] = 214;
assign a2i[86] = 177;
assign a2i[87] = 127;
assign a2i[88] = 254;
assign a2i[89] = 225;
assign a2i[90] = 223;
assign a2i[91] = 163;
assign a2i[92] = 91;
assign a2i[93] = 182;
assign a2i[94] = 113;
assign a2i[95] = 226;
assign a2i[96] = 217;
assign a2i[97] = 175;
assign a2i[98] = 67;
assign a2i[99] = 134;
assign a2i[100] = 17;
assign a2i[101] = 34;
assign a2i[102] = 68;
assign a2i[103] = 136;
assign a2i[104] = 13;
assign a2i[105] = 26;
assign a2i[106] = 52;
assign a2i[107] = 104;
assign a2i[108] = 208;
assign a2i[109] = 189;
assign a2i[110] = 103;
assign a2i[111] = 206;
assign a2i[112] = 129;
assign a2i[113] = 31;
assign a2i[114] = 62;
assign a2i[115] = 124;
assign a2i[116] = 248;
assign a2i[117] = 237;
assign a2i[118] = 199;
assign a2i[119] = 147;
assign a2i[120] = 59;
assign a2i[121] = 118;
assign a2i[122] = 236;
assign a2i[123] = 197;
assign a2i[124] = 151;
assign a2i[125] = 51;
assign a2i[126] = 102;
assign a2i[127] = 204;
assign a2i[128] = 133;
assign a2i[129] = 23;
assign a2i[130] = 46;
assign a2i[131] = 92;
assign a2i[132] = 184;
assign a2i[133] = 109;
assign a2i[134] = 218;
assign a2i[135] = 169;
assign a2i[136] = 79;
assign a2i[137] = 158;
assign a2i[138] = 33;
assign a2i[139] = 66;
assign a2i[140] = 132;
assign a2i[141] = 21;
assign a2i[142] = 42;
assign a2i[143] = 84;
assign a2i[144] = 168;
assign a2i[145] = 77;
assign a2i[146] = 154;
assign a2i[147] = 41;
assign a2i[148] = 82;
assign a2i[149] = 164;
assign a2i[150] = 85;
assign a2i[151] = 170;
assign a2i[152] = 73;
assign a2i[153] = 146;
assign a2i[154] = 57;
assign a2i[155] = 114;
assign a2i[156] = 228;
assign a2i[157] = 213;
assign a2i[158] = 183;
assign a2i[159] = 115;
assign a2i[160] = 230;
assign a2i[161] = 209;
assign a2i[162] = 191;
assign a2i[163] = 99;
assign a2i[164] = 198;
assign a2i[165] = 145;
assign a2i[166] = 63;
assign a2i[167] = 126;
assign a2i[168] = 252;
assign a2i[169] = 229;
assign a2i[170] = 215;
assign a2i[171] = 179;
assign a2i[172] = 123;
assign a2i[173] = 246;
assign a2i[174] = 241;
assign a2i[175] = 255;
assign a2i[176] = 227;
assign a2i[177] = 219;
assign a2i[178] = 171;
assign a2i[179] = 75;
assign a2i[180] = 150;
assign a2i[181] = 49;
assign a2i[182] = 98;
assign a2i[183] = 196;
assign a2i[184] = 149;
assign a2i[185] = 55;
assign a2i[186] = 110;
assign a2i[187] = 220;
assign a2i[188] = 165;
assign a2i[189] = 87;
assign a2i[190] = 174;
assign a2i[191] = 65;
assign a2i[192] = 130;
assign a2i[193] = 25;
assign a2i[194] = 50;
assign a2i[195] = 100;
assign a2i[196] = 200;
assign a2i[197] = 141;
assign a2i[198] = 7;
assign a2i[199] = 14;
assign a2i[200] = 28;
assign a2i[201] = 56;
assign a2i[202] = 112;
assign a2i[203] = 224;
assign a2i[204] = 221;
assign a2i[205] = 167;
assign a2i[206] = 83;
assign a2i[207] = 166;
assign a2i[208] = 81;
assign a2i[209] = 162;
assign a2i[210] = 89;
assign a2i[211] = 178;
assign a2i[212] = 121;
assign a2i[213] = 242;
assign a2i[214] = 249;
assign a2i[215] = 239;
assign a2i[216] = 195;
assign a2i[217] = 155;
assign a2i[218] = 43;
assign a2i[219] = 86;
assign a2i[220] = 172;
assign a2i[221] = 69;
assign a2i[222] = 138;
assign a2i[223] = 9;
assign a2i[224] = 18;
assign a2i[225] = 36;
assign a2i[226] = 72;
assign a2i[227] = 144;
assign a2i[228] = 61;
assign a2i[229] = 122;
assign a2i[230] = 244;
assign a2i[231] = 245;
assign a2i[232] = 247;
assign a2i[233] = 243;
assign a2i[234] = 251;
assign a2i[235] = 235;
assign a2i[236] = 203;
assign a2i[237] = 139;
assign a2i[238] = 11;
assign a2i[239] = 22;
assign a2i[240] = 44;
assign a2i[241] = 88;
assign a2i[242] = 176;
assign a2i[243] = 125;
assign a2i[244] = 250;
assign a2i[245] = 233;
assign a2i[246] = 207;
assign a2i[247] = 131;
assign a2i[248] = 27;
assign a2i[249] = 54;
assign a2i[250] = 108;
assign a2i[251] = 216;
assign a2i[252] = 173;
assign a2i[253] = 71;
assign a2i[254] = 142;
assign a2i[255] = 1;
assign i2a[0] = 0;
assign i2a[1] = 0;
assign i2a[2] = 1;
assign i2a[3] = 25;
assign i2a[4] = 2;
assign i2a[5] = 50;
assign i2a[6] = 26;
assign i2a[7] = 198;
assign i2a[8] = 3;
assign i2a[9] = 223;
assign i2a[10] = 51;
assign i2a[11] = 238;
assign i2a[12] = 27;
assign i2a[13] = 104;
assign i2a[14] = 199;
assign i2a[15] = 75;
assign i2a[16] = 4;
assign i2a[17] = 100;
assign i2a[18] = 224;
assign i2a[19] = 14;
assign i2a[20] = 52;
assign i2a[21] = 141;
assign i2a[22] = 239;
assign i2a[23] = 129;
assign i2a[24] = 28;
assign i2a[25] = 193;
assign i2a[26] = 105;
assign i2a[27] = 248;
assign i2a[28] = 200;
assign i2a[29] = 8;
assign i2a[30] = 76;
assign i2a[31] = 113;
assign i2a[32] = 5;
assign i2a[33] = 138;
assign i2a[34] = 101;
assign i2a[35] = 47;
assign i2a[36] = 225;
assign i2a[37] = 36;
assign i2a[38] = 15;
assign i2a[39] = 33;
assign i2a[40] = 53;
assign i2a[41] = 147;
assign i2a[42] = 142;
assign i2a[43] = 218;
assign i2a[44] = 240;
assign i2a[45] = 18;
assign i2a[46] = 130;
assign i2a[47] = 69;
assign i2a[48] = 29;
assign i2a[49] = 181;
assign i2a[50] = 194;
assign i2a[51] = 125;
assign i2a[52] = 106;
assign i2a[53] = 39;
assign i2a[54] = 249;
assign i2a[55] = 185;
assign i2a[56] = 201;
assign i2a[57] = 154;
assign i2a[58] = 9;
assign i2a[59] = 120;
assign i2a[60] = 77;
assign i2a[61] = 228;
assign i2a[62] = 114;
assign i2a[63] = 166;
assign i2a[64] = 6;
assign i2a[65] = 191;
assign i2a[66] = 139;
assign i2a[67] = 98;
assign i2a[68] = 102;
assign i2a[69] = 221;
assign i2a[70] = 48;
assign i2a[71] = 253;
assign i2a[72] = 226;
assign i2a[73] = 152;
assign i2a[74] = 37;
assign i2a[75] = 179;
assign i2a[76] = 16;
assign i2a[77] = 145;
assign i2a[78] = 34;
assign i2a[79] = 136;
assign i2a[80] = 54;
assign i2a[81] = 208;
assign i2a[82] = 148;
assign i2a[83] = 206;
assign i2a[84] = 143;
assign i2a[85] = 150;
assign i2a[86] = 219;
assign i2a[87] = 189;
assign i2a[88] = 241;
assign i2a[89] = 210;
assign i2a[90] = 19;
assign i2a[91] = 92;
assign i2a[92] = 131;
assign i2a[93] = 56;
assign i2a[94] = 70;
assign i2a[95] = 64;
assign i2a[96] = 30;
assign i2a[97] = 66;
assign i2a[98] = 182;
assign i2a[99] = 163;
assign i2a[100] = 195;
assign i2a[101] = 72;
assign i2a[102] = 126;
assign i2a[103] = 110;
assign i2a[104] = 107;
assign i2a[105] = 58;
assign i2a[106] = 40;
assign i2a[107] = 84;
assign i2a[108] = 250;
assign i2a[109] = 133;
assign i2a[110] = 186;
assign i2a[111] = 61;
assign i2a[112] = 202;
assign i2a[113] = 94;
assign i2a[114] = 155;
assign i2a[115] = 159;
assign i2a[116] = 10;
assign i2a[117] = 21;
assign i2a[118] = 121;
assign i2a[119] = 43;
assign i2a[120] = 78;
assign i2a[121] = 212;
assign i2a[122] = 229;
assign i2a[123] = 172;
assign i2a[124] = 115;
assign i2a[125] = 243;
assign i2a[126] = 167;
assign i2a[127] = 87;
assign i2a[128] = 7;
assign i2a[129] = 112;
assign i2a[130] = 192;
assign i2a[131] = 247;
assign i2a[132] = 140;
assign i2a[133] = 128;
assign i2a[134] = 99;
assign i2a[135] = 13;
assign i2a[136] = 103;
assign i2a[137] = 74;
assign i2a[138] = 222;
assign i2a[139] = 237;
assign i2a[140] = 49;
assign i2a[141] = 197;
assign i2a[142] = 254;
assign i2a[143] = 24;
assign i2a[144] = 227;
assign i2a[145] = 165;
assign i2a[146] = 153;
assign i2a[147] = 119;
assign i2a[148] = 38;
assign i2a[149] = 184;
assign i2a[150] = 180;
assign i2a[151] = 124;
assign i2a[152] = 17;
assign i2a[153] = 68;
assign i2a[154] = 146;
assign i2a[155] = 217;
assign i2a[156] = 35;
assign i2a[157] = 32;
assign i2a[158] = 137;
assign i2a[159] = 46;
assign i2a[160] = 55;
assign i2a[161] = 63;
assign i2a[162] = 209;
assign i2a[163] = 91;
assign i2a[164] = 149;
assign i2a[165] = 188;
assign i2a[166] = 207;
assign i2a[167] = 205;
assign i2a[168] = 144;
assign i2a[169] = 135;
assign i2a[170] = 151;
assign i2a[171] = 178;
assign i2a[172] = 220;
assign i2a[173] = 252;
assign i2a[174] = 190;
assign i2a[175] = 97;
assign i2a[176] = 242;
assign i2a[177] = 86;
assign i2a[178] = 211;
assign i2a[179] = 171;
assign i2a[180] = 20;
assign i2a[181] = 42;
assign i2a[182] = 93;
assign i2a[183] = 158;
assign i2a[184] = 132;
assign i2a[185] = 60;
assign i2a[186] = 57;
assign i2a[187] = 83;
assign i2a[188] = 71;
assign i2a[189] = 109;
assign i2a[190] = 65;
assign i2a[191] = 162;
assign i2a[192] = 31;
assign i2a[193] = 45;
assign i2a[194] = 67;
assign i2a[195] = 216;
assign i2a[196] = 183;
assign i2a[197] = 123;
assign i2a[198] = 164;
assign i2a[199] = 118;
assign i2a[200] = 196;
assign i2a[201] = 23;
assign i2a[202] = 73;
assign i2a[203] = 236;
assign i2a[204] = 127;
assign i2a[205] = 12;
assign i2a[206] = 111;
assign i2a[207] = 246;
assign i2a[208] = 108;
assign i2a[209] = 161;
assign i2a[210] = 59;
assign i2a[211] = 82;
assign i2a[212] = 41;
assign i2a[213] = 157;
assign i2a[214] = 85;
assign i2a[215] = 170;
assign i2a[216] = 251;
assign i2a[217] = 96;
assign i2a[218] = 134;
assign i2a[219] = 177;
assign i2a[220] = 187;
assign i2a[221] = 204;
assign i2a[222] = 62;
assign i2a[223] = 90;
assign i2a[224] = 203;
assign i2a[225] = 89;
assign i2a[226] = 95;
assign i2a[227] = 176;
assign i2a[228] = 156;
assign i2a[229] = 169;
assign i2a[230] = 160;
assign i2a[231] = 81;
assign i2a[232] = 11;
assign i2a[233] = 245;
assign i2a[234] = 22;
assign i2a[235] = 235;
assign i2a[236] = 122;
assign i2a[237] = 117;
assign i2a[238] = 44;
assign i2a[239] = 215;
assign i2a[240] = 79;
assign i2a[241] = 174;
assign i2a[242] = 213;
assign i2a[243] = 233;
assign i2a[244] = 230;
assign i2a[245] = 231;
assign i2a[246] = 173;
assign i2a[247] = 232;
assign i2a[248] = 116;
assign i2a[249] = 214;
assign i2a[250] = 244;
assign i2a[251] = 234;
assign i2a[252] = 168;
assign i2a[253] = 80;
assign i2a[254] = 88;
assign i2a[255] = 175;

endmodule


