
//////
module top (clk, reset, go, Dec_EncBar, PRNG, inp_share0, inp_share1,
    key, out_share0, out_share1, done);

    input clk, reset, go, Dec_EncBar;    // Dec_EncBar = 0 -> Encryption Mode
    input  [279 : 0] PRNG;
    input  [63  : 0] inp_share0, inp_share1;
    input  [127 : 0] key;
    output [63  : 0] out_share0, out_share1;
    output done;

    wire cnt_rst, cnt_en, start_path, inv1, inv2;
    wire [3 : 0] round_num;


    Datapath inst_DP (clk, cnt_rst, cnt_en, start_path, inv1, inv2, Dec_EncBar,
        PRNG, inp_share0, inp_share1, key, out_share0, out_share1, round_num);

    Controller inst_CTRL (clk, reset, go, round_num, cnt_rst, cnt_en, start_path,
        inv1, inv2, done);

endmodule


////////////////---Controller-----////////////////

/// Note: this controller is not optimum, in terms of Gate count.

module Controller(clk, reset, go, round_num,
    cnt_rst, cnt_en, start_path, inv1, inv2, done);

    input clk, reset, go;
    input [3 : 0] round_num;
    output reg cnt_rst, cnt_en, start_path, inv1, inv2, done;

    parameter [2 : 0] idle = 0, start = 1, half_round = 2, halt_state = 3, intermed_state = 4,
    final_round = 5, Finish = 6;

    reg [2:0] present_st;

    always @ (posedge clk)
    begin
        if (reset)
            present_st <= idle;
        else case (present_st)
            idle:
            begin
                cnt_rst <= 1 ; cnt_en <= 0 ; start_path <= 1;
                inv1 <= 0 ; inv2 <= 0 ; done <= 0;
                if (go == 1'b1)
                    begin
                        present_st <= start ;
                    end
                else begin
                    present_st <= idle ;
                end
            end

            start:
            begin
                cnt_rst <= 0 ; cnt_en <= 1 ; start_path <= 1;
                inv1 <= 0 ; inv2 <= 0 ; done <= 0;
                present_st <= half_round;
            end

            half_round:
            begin
                if (round_num < 5)
                    begin
                        cnt_rst <= 0 ; cnt_en <= 1 ; start_path <= 0;
                        inv1 <= 0 ; inv2 <= 0 ; done <= 0;
                        present_st <= half_round ;
                    end
                else
                    present_st <= halt_state ;
            end

            halt_state:
            begin
                cnt_rst <= 0 ; cnt_en <= 0 ; start_path <= 0;
                inv1 <= 0 ; inv2 <= 0 ; done <= 0;
                present_st <= intermed_state ;
            end

            intermed_state :
            begin
                cnt_rst <= 0 ; cnt_en <= 1 ; start_path <= 0;
                inv1 <= 1 ; inv2 <= 0 ; done <= 0;
                present_st <= final_round ;
            end

            final_round:
            begin

                if (round_num == 11)
                    begin
                        present_st <= Finish;
                    end
                else
                    begin
                        cnt_rst <= 0 ; cnt_en <= 1 ; start_path <= 0;
                        inv1 <= 1 ; inv2 <= 1 ; done <= 0;
                        present_st <= final_round ;
                    end
            end

            Finish:
            begin
                done <= 1;
                cnt_en <= 0 ;
                present_st <= idle ;
            end

            default:
            present_st <= idle;
        endcase
    end

endmodule





////////////////----Datapath-----////////////////


module Datapath(clk, cnt_rst, cnt_en, start_path, inv1_ctrl, inv2_ctrl, Dec_EncBar, PRNG,
    inp_share0, inp_share1, key, out_share0, out_share1, round_num);

    input clk, cnt_rst, Dec_EncBar, cnt_en, start_path, inv1_ctrl, inv2_ctrl;
    input  [279 : 0] PRNG;
    input  [127 : 0] key;
    input  [63  : 0] inp_share0, inp_share1;
    output [63  : 0] out_share0, out_share1;
    output [3   : 0] round_num;
    
    wire [63 : 0] k_0, k_0_prim, k_1, k0_from_mux, key_xor_inp_share0,
    SR_inv0_out, SR_inv1_out, Mp_share0_to_mux, Mp_share1_to_mux, SR_out_share0,
    SR_out_share1, RC, k1_xor_RC, SR_share0_xor_RCk1XORed, Aout_b4SB_share0,
    Aout_b4SB_share1, SB_in0, SB_in1, SB_out0, SB_out1, Aout_afterSB_share0, 
    Aout_afterSB_share1, A0_XORed_RCk1, SRinv_to_Mux_share0, SRinv_to_Mux_share1;
	
    wire [127 : 0] mux_starter_out, mux_to_SB, mux_to_Mp;
    
    Counter_4bit inst_CNT (clk, cnt_rst, cnt_en, round_num);

    Key_extraction inst_key_extractor (key, k_0, k_0_prim, k_1);

    RC_Sel inst_RC_selectoin (round_num, Dec_EncBar, RC);
    
    assign k1_xor_RC = k_1 ^ RC;

    Mux_2to1 #(64) inst_mux_key (k_0_prim, k_0, (start_path ^ Dec_EncBar), k0_from_mux);

    assign key_xor_inp_share0 = inp_share0 ^ k0_from_mux;

    SR_inv inst_SRinv_starter_share0 (key_xor_inp_share0, SR_inv0_out);
    SR_inv inst_SRinv_starter_share1 (inp_share1, SR_inv1_out);

    Mux_2to1 #(128) inst_starter_mux ({Mp_share1_to_mux, Mp_share0_to_mux},
        {SR_inv1_out, SR_inv0_out}, start_path, mux_starter_out);

    SR inst_SR_share0 (mux_starter_out[63 : 0], SR_out_share0);
    SR inst_SR_share1 (mux_starter_out[127 : 64], SR_out_share1);

    assign SR_share0_xor_RCk1XORed = SR_out_share0 ^ k1_xor_RC;

    A_share0 inst_A_up_share0 (mux_starter_out[63 : 0], Aout_b4SB_share0);
    A_share1 inst_A_up_share1 (mux_starter_out[127 : 64], Aout_b4SB_share1);

    Mux_2to1 #(128) inst_mux_before_SB ({SR_out_share1, SR_share0_xor_RCk1XORed},
        {Aout_b4SB_share1, Aout_b4SB_share0}, inv1_ctrl, mux_to_SB);

    assign SB_in0 = mux_to_SB[63  : 0];
    assign SB_in1 = mux_to_SB[127 : 64];

    //// Sbox instantiation
    /// SB numbers : 3, 7, 11, 15 -> OTSM

    

    /// other SBs -> TSMp
    TSMp_PRINCE_SB inst_S0  (clk, PRNG[13  :  0 ], SB_in0[3  :  0], SB_in1[3  :  0], SB_out0[3  :  0], SB_out1[3  :  0]);  
    TSMp_PRINCE_SB inst_S1  (clk, PRNG[27  :  14], SB_in0[7  :  4], SB_in1[7  :  4], SB_out0[7  :  4], SB_out1[7  :  4]);
    TSMp_PRINCE_SB inst_S2  (clk, PRNG[41  :  28], SB_in0[11 :  8], SB_in1[11 :  8], SB_out0[11 :  8], SB_out1[11 :  8]);
    OTSM_PRINCE_SB inst_S3  (clk, PRNG[69  :  42], SB_in0[15 : 12], SB_in1[15 : 12], SB_out0[15 : 12], SB_out1[15 : 12]); // SB 12 -> OTSM
    TSMp_PRINCE_SB inst_S4  (clk, PRNG[83  :  70], SB_in0[19 : 16], SB_in1[19 : 16], SB_out0[19 : 16], SB_out1[19 : 16]);
    TSMp_PRINCE_SB inst_S5  (clk, PRNG[97  :  84], SB_in0[23 : 20], SB_in1[23 : 20], SB_out0[23 : 20], SB_out1[23 : 20]);
    TSMp_PRINCE_SB inst_S6  (clk, PRNG[111 :  98], SB_in0[27 : 24], SB_in1[27 : 24], SB_out0[27 : 24], SB_out1[27 : 24]);
    OTSM_PRINCE_SB inst_S7  (clk, PRNG[139 : 112], SB_in0[31 : 28], SB_in1[31 : 28], SB_out0[31 : 28], SB_out1[31 : 28]); // SB 8 -> OTSM
    TSMp_PRINCE_SB inst_S8  (clk, PRNG[153 : 140], SB_in0[35 : 32], SB_in1[35 : 32], SB_out0[35 : 32], SB_out1[35 : 32]);
    TSMp_PRINCE_SB inst_S9  (clk, PRNG[167 : 154], SB_in0[39 : 36], SB_in1[39 : 36], SB_out0[39 : 36], SB_out1[39 : 36]);
    TSMp_PRINCE_SB inst_S10 (clk, PRNG[181 : 168], SB_in0[43 : 40], SB_in1[43 : 40], SB_out0[43 : 40], SB_out1[43 : 40]);
    OTSM_PRINCE_SB inst_S11 (clk, PRNG[209 : 182], SB_in0[47 : 44], SB_in1[47 : 44], SB_out0[47 : 44], SB_out1[47 : 44]); // SB 4 -> OTSM
    TSMp_PRINCE_SB inst_S12 (clk, PRNG[223 : 210], SB_in0[51 : 48], SB_in1[51 : 48], SB_out0[51 : 48], SB_out1[51 : 48]);
    TSMp_PRINCE_SB inst_S13 (clk, PRNG[237 : 224], SB_in0[55 : 52], SB_in1[55 : 52], SB_out0[55 : 52], SB_out1[55 : 52]);
    TSMp_PRINCE_SB inst_S14 (clk, PRNG[251 : 238], SB_in0[59 : 56], SB_in1[59 : 56], SB_out0[59 : 56], SB_out1[59 : 56]); 
    OTSM_PRINCE_SB inst_S15 (clk, PRNG[279 : 252], SB_in0[63 : 60], SB_in1[63 : 60], SB_out0[63 : 60], SB_out1[63 : 60]); // SB 0 -> OTSM



    A_share0 inst_A_down_share0 (SB_out0, Aout_afterSB_share0);
    A_share1 inst_A_down_share1 (SB_out1, Aout_afterSB_share1);

    assign A0_XORed_RCk1 = Aout_afterSB_share0 ^ k1_xor_RC;

    SR_inv inst_SRinv_share0 (A0_XORed_RCk1,       SRinv_to_Mux_share0);
    SR_inv inst_SRinv_share1 (Aout_afterSB_share1, SRinv_to_Mux_share1);
    
    Mux_2to1 #(128) inst_mux_before_Mp ({SB_out1, SB_out0}, {SRinv_to_Mux_share1, SRinv_to_Mux_share0},
        inv2_ctrl, mux_to_Mp);
    
    M_prime inst_Mp_share0 (mux_to_Mp[63  : 0],   Mp_share0_to_mux);
    M_prime inst_Mp_share1 (mux_to_Mp[127 : 64], Mp_share1_to_mux);
    
    assign out_share0 = A0_XORed_RCk1 ^ k0_from_mux;
    assign out_share1 = Aout_afterSB_share1;
    
endmodule



////////////////----- A(.) ------////////////////

module A0 (state, out);
    input  [3 : 0] state;
    output [3 : 0] out;

    assign out[0] = 1 ^ state [1];
    assign out[1] = state [0] ^ state [1] ^ state [2];
    assign out[2] = 1 ^ state [3];
    assign out[3] = state [2];
    
endmodule


////////////////-----A(.)-----////////////////


module A1 (state, out);
    input  [3 : 0] state;
    output [3 : 0] out;

    assign out[0] = state [1];
    assign out[1] = state [0] ^ state [1] ^ state [2];
    assign out[2] = state [3];
    assign out[3] = state [2];
    
endmodule


////////////////-----A(.) , share0------////////////////


module A_share0 (state, out);
    input  [63 : 0] state;
    output [63 : 0] out;

    genvar i;
    generate
        for (i = 0 ; i < 16 ; i = i + 1) begin : A_blks_sh0
            A0 inst_in_loop (state[(4 * i) + 3 : 4*i], 
			out[(4 * i) + 3 : 4*i]);
        end
    endgenerate

endmodule 


////////////////----- A(.) , share1 ------////////////////


module A_share1 (state, out);
    input [63 : 0] state;
    output [63 : 0] out;

    genvar i;
    generate
        for (i = 0 ; i < 16 ; i = i + 1) begin : A_blks_sh1
            A1 inst_in_loop (state[(4 * i) + 3 : 4*i], 
			out[(4 * i) + 3 : 4*i]);
        end
    endgenerate

endmodule


////////////////----- Multiplexer, 2 to 1-----////////////////


module Mux_2to1 #(parameter N = 64) (a, b, sel, y);
    input [N - 1 : 0] a, b;
    input sel;
    output [N - 1 : 0] y;
    
    assign y = (sel) ? (b) : (a) ;
    
endmodule 


////////////////----- 4-bit counter------////////////////


module Counter_4bit(clk, rst, start, y);
    input clk, rst, start;
    output [3 : 0] y;
    
    reg [3 : 0] r_y;
    assign y = r_y ;
    
    always @ (posedge clk) begin
        if (rst)
            r_y <= 4'b0010;
        else begin
            if (start)
                r_y <= r_y + 1;
            else
                r_y <= r_y;
        end    
    end
endmodule


////////////////----- Key extractor -----////////////////


module Key_extraction(key, k_0, k_0_prim, k_1);
    input [127 : 0] key;
    output [63 : 0] k_0, k_0_prim, k_1;

assign k_0 = key[127 : 64];
assign k_0_prim = {key[64], key[127 : 66], key[65] ^ key[127]};
assign k_1 = key[63 : 0];

endmodule


////////////////----- M_prime function -----////////////////


module M_prime (state, out);
    input  [63 : 0] state;
    output [63 : 0] out;

    assign out =
    {
        state[59] ^ state[55] ^ state[51],
        state[62] ^ state[54] ^ state[50],
        state[61] ^ state[57] ^ state[49],
        state[60] ^ state[56] ^ state[52],
        
        state[63] ^ state[59] ^ state[55],
        state[58] ^ state[54] ^ state[50],
        state[61] ^ state[53] ^ state[49],
        state[60] ^ state[56] ^ state[48],
        
        state[63] ^ state[59] ^ state[51],
        state[62] ^ state[58] ^ state[54],
        state[57] ^ state[53] ^ state[49],
        state[60] ^ state[52] ^ state[48],
        
        state[63] ^ state[55] ^ state[51],
        state[62] ^ state[58] ^ state[50],
        state[61] ^ state[57] ^ state[53],
        state[56] ^ state[52] ^ state[48],
        
        state[47] ^ state[43] ^ state[39],
        state[42] ^ state[38] ^ state[34],
        state[45] ^ state[37] ^ state[33],
        state[44] ^ state[40] ^ state[32],
        
        state[47] ^ state[43] ^ state[35],
        state[46] ^ state[42] ^ state[38],
        state[41] ^ state[37] ^ state[33],
        state[44] ^ state[36] ^ state[32],
        
        state[47] ^ state[39] ^ state[35],
        state[46] ^ state[42] ^ state[34],
        state[45] ^ state[41] ^ state[37],
        state[40] ^ state[36] ^ state[32],
        
        state[43] ^ state[39] ^ state[35],
        state[46] ^ state[38] ^ state[34],
        state[45] ^ state[41] ^ state[33],
        state[44] ^ state[40] ^ state[36],
        
        state[31] ^ state[27] ^ state[23],
        state[26] ^ state[22] ^ state[18],
        state[29] ^ state[21] ^ state[17],
        state[28] ^ state[24] ^ state[16],
        
        state[31] ^ state[27] ^ state[19],
        state[30] ^ state[26] ^ state[22],
        state[25] ^ state[21] ^ state[17],
        state[28] ^ state[20] ^ state[16],
        
        state[31] ^ state[23] ^ state[19],
        state[30] ^ state[26] ^ state[18],
        state[29] ^ state[25] ^ state[21],
        state[24] ^ state[20] ^ state[16],
        
        state[27] ^ state[23] ^ state[19],
        state[30] ^ state[22] ^ state[18],
        state[29] ^ state[25] ^ state[17],
        state[28] ^ state[24] ^ state[20],
        
        state[11] ^ state[7]  ^ state[3],
        state[14] ^ state[6]  ^ state[2],
        state[13] ^ state[9]  ^ state[1],
        state[12] ^ state[8]  ^ state[4],
        
        state[15] ^ state[11] ^ state[7],
        state[10] ^ state[6]  ^ state[2],
        state[13] ^ state[5]  ^ state[1],
        state[12] ^ state[8]  ^ state[0],
        
        state[15] ^ state[11] ^ state[3],
        state[14] ^ state[10] ^ state[6],
        state[9]  ^ state[5]  ^ state[1],
        state[12] ^ state[4]  ^ state[0],
        
        state[15] ^ state[7]  ^ state[3],
        state[14] ^ state[10] ^ state[2],
        state[13] ^ state[9]  ^ state[5],
        state[8]  ^ state[4]  ^ state[0]
    };

endmodule


////////////////----- RC selector-----////////////////


module RC_Sel (sel, enc_dec, RC);
    input [3 : 0] sel;
    input enc_dec;
    output reg [63 : 0] RC;

    wire [3 : 0] roundXORenc;
    assign roundXORenc = sel ^ {4{enc_dec}};

    always @(*) begin
        case (roundXORenc)
            4'b0010: RC = 64'h0000000000000000;
            4'b0011: RC = 64'h13198A2E03707344;
            4'b0100: RC = 64'hA4093822299F31D0;
            4'b0101: RC = 64'h082EFA98EC4E6C89;
            4'b0110: RC = 64'h452821E638D01377;
            4'b0111: RC = 64'hBE5466CF34E90C6C;
            4'b1000: RC = 64'h7EF84F78FD955CB1;
            4'b1001: RC = 64'h85840851F1AC43AA;
            4'b1010: RC = 64'hC882D32F25323C54;
            4'b1011: RC = 64'h64A51195E0E3610D;
            4'b1100: RC = 64'hD3B5A399CA0C2399;
            4'b1101: RC = 64'hC0AC29B7C97C50DD;
            default: RC = 64'h0000000000000000; 
        endcase
    end
endmodule


////////////////-----------////////////////


module SR(state, out);
    input  [63 : 0] state;
    output [63 : 0] out;

    assign out =
    {
    state[63 : 60], 
    state[43 : 40],
    state[23 : 20], 
    state[3  : 0],
    state[47 : 44], 
    state[27 : 24],
    state[7  :  4], 
    state[51 : 48],
    state[31 : 28], 
    state[11 : 8],
    state[55 : 52], 
    state[35 : 32],
    state[15 : 12], 
    state[59 : 56],
    state[39 : 36], 
    state[19 : 16]
    };

endmodule


////////////////-----------////////////////


module SR_inv(state, out);
    input  [63 : 0] state;
    output [63 : 0] out;

    assign out =
    {
    state[63 : 60], 
    state[11 :  8],
    state[23 : 20], 
    state[35 : 32],
    state[47 : 44], 
    state[59 : 56],
    state[7  :  4], 
    state[19 : 16],
    state[31 : 28], 
    state[43 : 40],
    state[55 : 52], 
    state[3  :  0],
    state[15 : 12], 
    state[27 : 24],
    state[39 : 36], 
    state[51 : 48]
    };

endmodule


//// Sboxes

////
////////////////
module OTSM_PRINCE_SB (clk, PRNG, inp_share0, inp_share1, F0, F1);
    input clk;
    input [27 : 0] PRNG;
    input [3 : 0] inp_share0, inp_share1;
    output [3 : 0] F0, F1;

    wire [13 : 0] r, rp;
    assign {r, rp} = PRNG;


    ///// ----- Share 0 -------
    wire a0_wire, b0_wire, c0_wire, d0_wire,
         a0_reg, b0_reg, c0_reg, d0_reg; 

    assign {a0_wire, b0_wire, c0_wire, d0_wire} = inp_share0;
    wire 
        ab_0, ac_0, ad_0, bc_0, bd_0, cd_0,
        abc_0, abd_0, acd_0, bcd_0;
    wire
        ab_balancer0, ab_balancer1, ac_balancer0, ac_balancer1,
        ad_balancer0, ad_balancer1, bc_balancer0, bc_balancer1,
        bd_balancer0, bd_balancer1, cd_balancer0, cd_balancer1;
    
    
    //// ------ Share 1 ------ 

    wire a1, b1, c1, d1,
        a1_reg, b1_reg, c1_reg, d1_reg,
        ab_1_reg, ac_1_reg, ad_1_reg, bc_1_reg,
        bd_1_reg, cd_1_reg, abc_1_reg, abd_1_reg,
        acd_1_reg, bcd_1_reg;
    
    
    Register #(4) inst_primary_inp0_reg({a0_wire, b0_wire, c0_wire, d0_wire}, clk, {a0_reg, b0_reg, c0_reg, d0_reg});

    assign ab_0 = (a0_reg & b1_reg) ^ (a1_reg & b0_reg);
    assign ab_balancer0 = (a0_wire & b0_wire) ^ (a0_wire & r[1]) ^ (b0_wire & r[0]) ^ r[4];
    assign ab_balancer1 = (a0_wire & rp[1]) ^ (b0_wire & rp[0]) ^ rp[4];

    assign ac_0 = (a0_reg & c1_reg) ^ (a1_reg & c0_reg);
    assign ac_balancer0 = (a0_wire & c0_wire) ^ (a0_wire & r[2]) ^ (c0_wire & r[0]) ^ r[5];
    assign ac_balancer1 = (a0_wire & rp[2]) ^ (c0_wire & rp[0]) ^ rp[5];

    assign ad_0 = (a0_reg & d1_reg) ^ (a1_reg & d0_reg);
    assign ad_balancer0 = (a0_wire & d0_wire) ^ (a0_wire & r[3]) ^ (d0_wire & r[0]) ^ r[6];
    assign ad_balancer1 = (a0_wire & rp[3]) ^ (d0_wire & rp[0]) ^ rp[6];

    assign bc_0 = (b0_reg & c1_reg) ^ (b1_reg & c0_reg);
    assign bc_balancer0 = (b0_wire & c0_wire) ^ (b0_wire & r[2]) ^ (c0_wire & r[1]) ^ r[7];
    assign bc_balancer1 = (b0_wire & rp[2]) ^ (c0_wire & rp[1]) ^ rp[7];

    assign bd_0 = (b0_reg & d1_reg) ^ (b1_reg & d0_reg);
    assign bd_balancer0 = (b0_wire & d0_wire) ^ (b0_wire & r[3]) ^ (d0_wire & r[1]) ^ r[8];
    assign bd_balancer1 = (b0_wire & rp[3]) ^ (d0_wire & rp[1]) ^ rp[8];

    assign cd_0 = (c0_reg & d1_reg) ^ (c1_reg & d0_reg);
    assign cd_balancer0 = (c0_wire & d0_wire) ^ (c0_wire & r[3]) ^ (d0_wire & r[2]) ^ r[9];
    assign cd_balancer1 = (c0_wire & rp[3]) ^ (d0_wire & rp[2]) ^ rp[9];


    assign abc_0 = (a0_reg & b0_reg & c1_reg) ^ (a0_reg & c0_reg & b1_reg) ^ (b0_reg & c0_reg & a1_reg) ^ (a0_reg & bc_1_reg) ^ (b0_reg & ac_1_reg) ^ (c0_reg & ab_1_reg);
    assign abc_balancer0 = (a0_wire & b0_wire & c0_wire) ^ (a0_wire & b0_wire & r[2]) ^ (a0_wire & c0_wire & r[1]) ^
        (b0_wire & c0_wire & r[0]) ^ (a0_wire & r[7]) ^ (b0_wire & r[5]) ^ (c0_wire & r[4]) ^ r[10];
    assign abc_balancer1 = (a0_wire & b0_wire & rp[2]) ^ (a0_wire & c0_wire & rp[1]) ^ (b0_wire & c0_wire & rp[0]) ^ 
        (a0_wire & rp[7]) ^ (b0_wire & rp[5]) ^ (c0_wire & rp[4]) ^ rp[10];

    assign abd_0 = (a0_reg & b0_reg & d1_reg) ^ (a0_reg & d0_reg & b1_reg) ^ (b0_reg & d0_reg & a1_reg) ^ (a0_reg & bd_1_reg) ^ (b0_reg & ad_1_reg) ^ (d0_reg & ab_1_reg);
    assign abd_balancer0 = (a0_wire & b0_wire & d0_wire) ^ (a0_wire & b0_wire & r[3]) ^  (a0_wire & d0_wire & r[1]) ^ 
        (b0_wire & d0_wire & r[0]) ^ (a0_wire & r[8]) ^ (b0_wire & r[6]) ^ (d0_wire & r[4]) ^ r[11];
    assign abd_balancer1 = (a0_wire & b0_wire & rp[3]) ^  (a0_wire & d0_wire & rp[1]) ^ (b0_wire & d0_wire & rp[0]) ^ 
        (a0_wire & rp[8]) ^ (b0_wire & rp[6]) ^ (d0_wire & rp[4]) ^ rp[11];

    assign acd_0 = (a0_reg & c0_reg & d1_reg) ^ (a0_reg & d0_reg & c1_reg) ^ (c0_reg & d0_reg & a1_reg) ^ (a0_reg & cd_1_reg) ^ (c0_reg & ad_1_reg) ^ (d0_reg & ac_1_reg);
    assign acd_balancer0 = (a0_wire & c0_wire & d0_wire) ^ (a0_wire & c0_wire & r[3]) ^  (a0_wire & d0_wire & r[2]) ^ 
        (c0_wire & d0_wire & r[0]) ^ (a0_wire & r[9]) ^ (c0_wire & r[6]) ^ (d0_wire & r[5]) ^ r[12];
    assign acd_balancer1 = (a0_wire & c0_wire & rp[3]) ^  (a0_wire & d0_wire & rp[2]) ^ (c0_wire & d0_wire & rp[0]) ^ 
        (a0_wire & rp[9]) ^ (c0_wire & rp[6]) ^ (d0_wire & rp[5]) ^ rp[12];

    assign bcd_0 = (b0_reg & c0_reg & d1_reg) ^ (b0_reg & d0_reg & c1_reg) ^ (c0_reg & d0_reg & b1_reg) ^ (b0_reg & cd_1_reg) ^ (c0_reg & bd_1_reg) ^ (d0_reg & bc_1_reg);
    assign bcd_balancer0 = (b0_wire & c0_wire & d0_wire) ^ (b0_wire & c0_wire & r[3]) ^  (b0_wire & d0_wire & r[2]) ^ 
        (c0_wire & d0_wire & r[1]) ^ (b0_wire & r[9]) ^ (c0_wire & r[8]) ^ (d0_wire & r[7]) ^ r[13];
    assign bcd_balancer1 = (b0_wire & c0_wire & rp[3]) ^  (b0_wire & d0_wire & rp[2]) ^ (c0_wire & d0_wire & rp[1]) ^ 
        (b0_wire & rp[9]) ^ (c0_wire & rp[8]) ^ (d0_wire & rp[7]) ^ rp[13];


    wire  [3 : 0] correction0, correction1, correction0_reg, correction1_reg;
    assign correction0[0] = r[0] ^ r[1] ^ ab_balancer0 ^ ad_balancer0 ^ bc_balancer0  ^ cd_balancer0 ^ bcd_balancer0;
    assign correction0[1]=  ac_balancer0 ^ bc_balancer0 ^ bd_balancer0 ^ abc_balancer0 ^ bcd_balancer0;
    assign correction0[2] = r[0] ^ r[3] ^ ac_balancer0 ^ ad_balancer0 ^ cd_balancer0  ^ abc_balancer0 ^ acd_balancer0;
    assign correction0[3] = r[0] ^ r[2] ^ ab_balancer0 ^ bc_balancer0 ^ abd_balancer0 ^ acd_balancer0 ^ bcd_balancer0;
    //
    assign correction1[0] = rp[0] ^ rp[1] ^ ab_balancer1 ^ ad_balancer1 ^ bc_balancer1  ^ cd_balancer1 ^ bcd_balancer1;
    assign correction1[1]=  ac_balancer1 ^ bc_balancer1 ^ bd_balancer1 ^ abc_balancer1 ^ bcd_balancer1;
    assign correction1[2] = rp[0] ^ rp[3] ^ ac_balancer1 ^ ad_balancer1 ^ cd_balancer1  ^ abc_balancer1 ^ acd_balancer1;
    assign correction1[3] = rp[0] ^ rp[2] ^ ab_balancer1 ^ bc_balancer1 ^ abd_balancer1 ^ acd_balancer1 ^ bcd_balancer1;
    
    Register #(4) inst_correction0_reg (correction0, clk, correction0_reg);
    Register #(4) inst_correction1_reg (correction1, clk, correction1_reg);
    
    assign F0[0] = a0_reg ^ b0_reg ^ ab_0 ^  ad_0 ^  bc_0 ^  cd_0 ^ bcd_0 ^ correction0_reg[0] ^ correction1_reg[0];
    assign F0[1] = ac_0   ^ bc_0   ^ bd_0 ^ abc_0 ^ bcd_0 ^                 correction0_reg[1] ^ correction1_reg[1];
    assign F0[2] = a0_reg ^ d0_reg ^ ac_0 ^  ad_0 ^  cd_0 ^ abc_0 ^ acd_0 ^ correction0_reg[2] ^ correction1_reg[2];
    assign F0[3] = a0_reg ^ c0_reg ^ ab_0 ^  bc_0 ^ abd_0 ^ acd_0 ^ bcd_0 ^ correction0_reg[3] ^ correction1_reg[3];




    assign {a1, b1, c1, d1} = inp_share1;
    assign a1_refreshed = a1 ^ r[0] ^ rp[0];
    assign b1_refreshed = b1 ^ r[1] ^ rp[1];
    assign c1_refreshed = c1 ^ r[2] ^ rp[2];
    assign d1_refreshed = d1 ^ r[3] ^ rp[3];
    
    Register #(1) inst_a1reg (a1_refreshed, clk, a1_reg);
    Register #(1) inst_b1reg (b1_refreshed, clk, b1_reg);
    Register #(1) inst_c1reg (c1_refreshed, clk, c1_reg);
    Register #(1) inst_d1reg (d1_refreshed, clk, d1_reg);

    Register #(1) inst_ab1_reg (a1 & b1 ^ r[4]  ^ rp[4],  clk, ab_1_reg);
    Register #(1) inst_ac1_reg (a1 & c1 ^ r[5] ^ rp[5], clk, ac_1_reg);
    Register #(1) inst_ad1_reg (a1 & d1 ^ r[6] ^ rp[6], clk, ad_1_reg);
    Register #(1) inst_bc1_reg (b1 & c1 ^ r[7] ^ rp[7], clk, bc_1_reg);
    Register #(1) inst_bd1_reg (b1 & d1 ^ r[8] ^ rp[8], clk, bd_1_reg);
    Register #(1) inst_cd1_reg (c1 & d1 ^ r[9] ^ rp[9], clk, cd_1_reg);
    
    Register #(1) inst_abc1_reg (a1 & b1 & c1 ^ r[10] ^ rp[10], clk, abc_1_reg);
    Register #(1) inst_abd1_reg (a1 & b1 & d1 ^ r[11] ^ rp[11], clk, abd_1_reg);
    Register #(1) inst_acd1_reg (a1 & c1 & d1 ^ r[12] ^ rp[12], clk, acd_1_reg);
    Register #(1) inst_bcd1_reg (b1 & c1 & d1 ^ r[13] ^ rp[13], clk, bcd_1_reg);

    assign F1[0] = 1'b1 ^ a1_reg ^ b1_reg ^ ab_1_reg ^ ad_1_reg ^ bc_1_reg ^ cd_1_reg ^ bcd_1_reg;
    assign F1[1] = 1'b1 ^ ac_1_reg ^ bc_1_reg ^ bd_1_reg ^ abc_1_reg ^ bcd_1_reg;
    assign F1[2] = a1_reg ^ d1_reg ^ ac_1_reg ^ ad_1_reg ^ cd_1_reg ^ abc_1_reg ^ acd_1_reg;
    assign F1[3] = 1'b1 ^ a1_reg ^ c1_reg ^ ab_1_reg ^ bc_1_reg ^ abd_1_reg ^ acd_1_reg ^ bcd_1_reg;

endmodule

//////////////////
module Register #(parameter N = 1)(in, clk, out);

    input [N-1 : 0] in;
    input clk;
    output reg [N-1 : 0] out;

    always @ (posedge clk) begin
	out <= in;
    end

endmodule


////
module TSMp_PRINCE_SB (clk, r, inp_share0, inp_share1, F0, F1);

    input clk;
    input [13 : 0] r;
    input [3 : 0] inp_share0, inp_share1;
    output [3 : 0] F0, F1;

    wire 
        a0_wire, b0_wire, c0_wire, d0_wire,
        a0_reg, b0_reg, c0_reg, d0_reg,
        ab_0, ac_0, ad_0, bc_0, bd_0, cd_0,
        abc_0, abd_0, acd_0, bcd_0;
    wire
        a1_reg, b1_reg, c1_reg, d1_reg,
        ab_1_reg, ac_1_reg, ad_1_reg, bc_1_reg, bd_1_reg, cd_1_reg,
        abc_1_reg, abd_1_reg, acd_1_reg, bcd_1_reg;
    wire
        ab_balancer, ac_balancer, ad_balancer, bc_balancer, 
        bd_balancer, cd_balancer, abc_balancer, abd_balancer, acd_balancer, bcd_balancer;

    
    ////////// ----- share 0
    assign {a0_wire, b0_wire, c0_wire, d0_wire} = inp_share0;
    Register #(4) inst_primary_input_share0 ({a0_wire, b0_wire, c0_wire, d0_wire}, clk, {a0_reg, b0_reg, c0_reg, d0_reg});

    assign ab_0 = (a0_reg & b1_reg) ^ (a1_reg & b0_reg);
    assign ab_balancer = (a0_wire & b0_wire) ^ (a0_wire & r[1]) ^ (b0_wire & r[0]) ^ r[4];

    assign ac_0 = (a0_reg & c1_reg) ^ (a1_reg & c0_reg);
    assign ac_balancer = (a0_wire & c0_wire) ^ (a0_wire & r[2]) ^ (c0_wire & r[0]) ^ r[5];

    assign ad_0 = (a0_reg & d1_reg) ^ (a1_reg & d0_reg);
    assign ad_balancer = (a0_wire & d0_wire) ^ (a0_wire & r[3]) ^ (d0_wire & r[0]) ^ r[6];

    assign bc_0 = (b0_reg & c1_reg) ^ (b1_reg & c0_reg);
    assign bc_balancer = (b0_wire & c0_wire) ^ (b0_wire & r[2]) ^ (c0_wire & r[1]) ^ r[7];

    assign bd_0 = (b0_reg & d1_reg) ^ (b1_reg & d0_reg);
    assign bd_balancer = (b0_wire & d0_wire) ^ (b0_wire & r[3]) ^ (d0_wire & r[1]) ^ r[8];

    assign cd_0 = (c0_reg & d1_reg) ^ (c1_reg & d0_reg);
    assign cd_balancer = (c0_wire & d0_wire) ^ (c0_wire & r[3]) ^ (d0_wire & r[2]) ^ r[9];

    assign abc_0 = (a0_reg & b0_reg & c1_reg) ^ (a0_reg & c0_reg & b1_reg) ^ (b0_reg & c0_reg & a1_reg) ^ (a0_reg & bc_1_reg) ^ (b0_reg & ac_1_reg) ^ (c0_reg & ab_1_reg);
    assign abc_balancer = (a0_wire & b0_wire & c0_wire) ^ (a0_wire & b0_wire & r[2]) ^ (a0_wire & c0_wire & r[1]) ^
        (b0_wire & c0_wire & r[0]) ^ (a0_wire & r[7]) ^ (b0_wire & r[5]) ^ (c0_wire & r[4]) ^ r[10];

    assign abd_0 = (a0_reg & b0_reg & d1_reg) ^ (a0_reg & d0_reg & b1_reg) ^ (b0_reg & d0_reg & a1_reg) ^ (a0_reg & bd_1_reg) ^ (b0_reg & ad_1_reg) ^ (d0_reg & ab_1_reg);
    assign abd_balancer = (a0_wire & b0_wire & d0_wire) ^ (a0_wire & b0_wire & r[3]) ^  (a0_wire & d0_wire & r[1]) ^ 
        (b0_wire & d0_wire & r[0]) ^ (a0_wire & r[8]) ^ (b0_wire & r[6]) ^ (d0_wire & r[4]) ^ r[11];

    assign acd_0 = (a0_reg & c0_reg & d1_reg) ^ (a0_reg & d0_reg & c1_reg) ^ (c0_reg & d0_reg & a1_reg) ^ (a0_reg & cd_1_reg) ^ (c0_reg & ad_1_reg) ^ (d0_reg & ac_1_reg);
    assign acd_balancer = (a0_wire & c0_wire & d0_wire) ^ (a0_wire & c0_wire & r[3]) ^  (a0_wire & d0_wire & r[2]) ^ 
        (c0_wire & d0_wire & r[0]) ^ (a0_wire & r[9]) ^ (c0_wire & r[6]) ^ (d0_wire & r[5]) ^ r[12];

    assign bcd_0 = (b0_reg & c0_reg & d1_reg) ^ (b0_reg & d0_reg & c1_reg) ^ (c0_reg & d0_reg & b1_reg) ^ (b0_reg & cd_1_reg) ^ (c0_reg & bd_1_reg) ^ (d0_reg & bc_1_reg);
    assign bcd_balancer = (b0_wire & c0_wire & d0_wire) ^ (b0_wire & c0_wire & r[3]) ^  (b0_wire & d0_wire & r[2]) ^ 
        (c0_wire & d0_wire & r[1]) ^ (b0_wire & r[9]) ^ (c0_wire & r[8]) ^ (d0_wire & r[7]) ^ r[13];

    wire  [3 : 0] balancer, balancer_reg;
    assign balancer[0] = r[0] ^ r[1] ^ ab_balancer ^ ad_balancer ^ bc_balancer  ^ cd_balancer ^ bcd_balancer;
    assign balancer[1]=  ac_balancer ^ bc_balancer ^ bd_balancer ^ abc_balancer ^ bcd_balancer;
    assign balancer[2] = r[0] ^ r[3] ^ ac_balancer ^ ad_balancer ^ cd_balancer  ^ abc_balancer ^ acd_balancer;
    assign balancer[3] = r[0] ^ r[2] ^ ab_balancer ^ bc_balancer ^ abd_balancer ^ acd_balancer ^ bcd_balancer;
    
    Register #(4) inst_balancer_reg (balancer, clk, balancer_reg);
    
    assign F0[0] = a0_reg ^ b0_reg ^ ab_0 ^  ad_0 ^  bc_0 ^  cd_0 ^ bcd_0 ^ balancer_reg[0];
    assign F0[1] = ac_0   ^ bc_0   ^ bd_0 ^ abc_0 ^ bcd_0 ^ balancer_reg[1];
    assign F0[2] = a0_reg ^ d0_reg ^ ac_0 ^  ad_0 ^  cd_0 ^ abc_0 ^ acd_0 ^ balancer_reg[2];
    assign F0[3] = a0_reg ^ c0_reg ^ ab_0 ^  bc_0 ^ abd_0 ^ acd_0 ^ bcd_0 ^ balancer_reg[3];


    ///////// -----  share1
    wire a1_wire, b1_wire, c1_wire, d1_wire,
        a1_refreshed, b1_refreshed, c1_refreshed, d1_refreshed,
        ab_1_wire, ac_1_wire, ad_1_wire, bc_1_wire, 
        bd_1_wire, cd_1_wire, abc_1_wire, abd_1_wire, acd_1_wire, 
        bcd_1_wire;

    assign a1_wire = inp_share1[3];
    assign b1_wire = inp_share1[2];
    assign c1_wire = inp_share1[1];
    assign d1_wire = inp_share1[0];

    assign a1_refreshed = a1_wire ^ r[0];
    assign b1_refreshed = b1_wire ^ r[1];
    assign c1_refreshed = c1_wire ^ r[2];
    assign d1_refreshed = d1_wire ^ r[3];

    Register #(4) inst_primary_input_share1 ({a1_refreshed, b1_refreshed, c1_refreshed, d1_refreshed}, clk, {a1_reg, b1_reg, c1_reg, d1_reg});

    assign ab_1_wire = (a1_wire & b1_wire) ^ r[4];
    Register #(1) inst_qb0 (ab_1_wire ,clk, ab_1_reg);

    assign ac_1_wire = (a1_wire & c1_wire) ^ r[5];
    Register #(1) inst_qb1 (ac_1_wire ,clk, ac_1_reg);

    assign ad_1_wire = (a1_wire & d1_wire) ^ r[6];
    Register #(1) inst_qb2 (ad_1_wire ,clk, ad_1_reg);

    assign bc_1_wire = (b1_wire & c1_wire) ^ r[7];
    Register #(1) inst_qb3 (bc_1_wire ,clk, bc_1_reg);

    assign bd_1_wire = (b1_wire & d1_wire) ^ r[8];
    Register #(1) inst_qb4 (bd_1_wire ,clk, bd_1_reg);

    assign cd_1_wire = (c1_wire & d1_wire) ^ r[9];
    Register #(1) inst_qb5 (cd_1_wire ,clk, cd_1_reg);

    assign abc_1_wire = (a1_wire & b1_wire & c1_wire) ^ r[10];
    Register #(1) inst_quad0 (abc_1_wire ,clk, abc_1_reg);

    assign abd_1_wire = (a1_wire & b1_wire & d1_wire) ^ r[11];
    Register #(1) inst_quad1 (abd_1_wire ,clk, abd_1_reg);

    assign acd_1_wire = (a1_wire & c1_wire & d1_wire) ^ r[12];
    Register #(1) inst_quad2 (acd_1_wire ,clk, acd_1_reg);

    assign bcd_1_wire = (b1_wire & c1_wire & d1_wire) ^ r[13];
    Register #(1) inst_quad3 (bcd_1_wire ,clk, bcd_1_reg);

    assign F1[0] = 1'b1 ^ a1_reg ^ b1_reg ^ ab_1_reg ^ ad_1_reg ^ bc_1_reg ^ cd_1_reg ^ bcd_1_reg;
    assign F1[1] = 1'b1 ^ ac_1_reg ^ bc_1_reg ^ bd_1_reg ^ abc_1_reg ^ bcd_1_reg;
    assign F1[2] = a1_reg ^ d1_reg ^ ac_1_reg ^ ad_1_reg ^ cd_1_reg ^ abc_1_reg ^ acd_1_reg;
    assign F1[3] = 1'b1 ^ a1_reg ^ c1_reg ^ ab_1_reg ^ bc_1_reg ^ abd_1_reg ^ acd_1_reg ^ bcd_1_reg;

endmodule
