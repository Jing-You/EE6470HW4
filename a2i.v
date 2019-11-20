// compute alpha^a
module a2i(
	input [7:0] a,
	output reg [7:0] i
);

always @* begin
	case(a):  // synopsys parallel_case
		0:i=1;
		1:i=2;
		2:i=4;
		3:i=8;
		4:i=16;
		5:i=32;
		6:i=64;
		7:i=128;
		8:i=29;
		9:i=58;
		10:i=116;
		11:i=232;
		12:i=205;
		13:i=135;
		14:i=19;
		15:i=38;
		16:i=76;
		17:i=152;
		18:i=45;
		19:i=90;
		20:i=180;
		21:i=117;
		22:i=234;
		23:i=201;
		24:i=143;
		25:i=3;
		26:i=6;
		27:i=12;
		28:i=24;
		29:i=48;
		30:i=96;
		31:i=192;
		32:i=157;
		33:i=39;
		34:i=78;
		35:i=156;
		36:i=37;
		37:i=74;
		38:i=148;
		39:i=53;
		40:i=106;
		41:i=212;
		42:i=181;
		43:i=119;
		44:i=238;
		45:i=193;
		46:i=159;
		47:i=35;
		48:i=70;
		49:i=140;
		50:i=5;
		51:i=10;
		52:i=20;
		53:i=40;
		54:i=80;
		55:i=160;
		56:i=93;
		57:i=186;
		58:i=105;
		59:i=210;
		60:i=185;
		61:i=111;
		62:i=222;
		63:i=161;
		64:i=95;
		65:i=190;
		66:i=97;
		67:i=194;
		68:i=153;
		69:i=47;
		70:i=94;
		71:i=188;
		72:i=101;
		73:i=202;
		74:i=137;
		75:i=15;
		76:i=30;
		77:i=60;
		78:i=120;
		79:i=240;
		80:i=253;
		81:i=231;
		82:i=211;
		83:i=187;
		84:i=107;
		85:i=214;
		86:i=177;
		87:i=127;
		88:i=254;
		89:i=225;
		90:i=223;
		91:i=163;
		92:i=91;
		93:i=182;
		94:i=113;
		95:i=226;
		96:i=217;
		97:i=175;
		98:i=67;
		99:i=134;
		100:i=17;
		101:i=34;
		102:i=68;
		103:i=136;
		104:i=13;
		105:i=26;
		106:i=52;
		107:i=104;
		108:i=208;
		109:i=189;
		110:i=103;
		111:i=206;
		112:i=129;
		113:i=31;
		114:i=62;
		115:i=124;
		116:i=248;
		117:i=237;
		118:i=199;
		119:i=147;
		120:i=59;
		121:i=118;
		122:i=236;
		123:i=197;
		124:i=151;
		125:i=51;
		126:i=102;
		127:i=204;
		128:i=133;
		129:i=23;
		130:i=46;
		131:i=92;
		132:i=184;
		133:i=109;
		134:i=218;
		135:i=169;
		136:i=79;
		137:i=158;
		138:i=33;
		139:i=66;
		140:i=132;
		141:i=21;
		142:i=42;
		143:i=84;
		144:i=168;
		145:i=77;
		146:i=154;
		147:i=41;
		148:i=82;
		149:i=164;
		150:i=85;
		151:i=170;
		152:i=73;
		153:i=146;
		154:i=57;
		155:i=114;
		156:i=228;
		157:i=213;
		158:i=183;
		159:i=115;
		160:i=230;
		161:i=209;
		162:i=191;
		163:i=99;
		164:i=198;
		165:i=145;
		166:i=63;
		167:i=126;
		168:i=252;
		169:i=229;
		170:i=215;
		171:i=179;
		172:i=123;
		173:i=246;
		174:i=241;
		175:i=255;
		176:i=227;
		177:i=219;
		178:i=171;
		179:i=75;
		180:i=150;
		181:i=49;
		182:i=98;
		183:i=196;
		184:i=149;
		185:i=55;
		186:i=110;
		187:i=220;
		188:i=165;
		189:i=87;
		190:i=174;
		191:i=65;
		192:i=130;
		193:i=25;
		194:i=50;
		195:i=100;
		196:i=200;
		197:i=141;
		198:i=7;
		199:i=14;
		200:i=28;
		201:i=56;
		202:i=112;
		203:i=224;
		204:i=221;
		205:i=167;
		206:i=83;
		207:i=166;
		208:i=81;
		209:i=162;
		210:i=89;
		211:i=178;
		212:i=121;
		213:i=242;
		214:i=249;
		215:i=239;
		216:i=195;
		217:i=155;
		218:i=43;
		219:i=86;
		220:i=172;
		221:i=69;
		222:i=138;
		223:i=9;
		224:i=18;
		225:i=36;
		226:i=72;
		227:i=144;
		228:i=61;
		229:i=122;
		230:i=244;
		231:i=245;
		232:i=247;
		233:i=243;
		234:i=251;
		235:i=235;
		236:i=203;
		237:i=139;
		238:i=11;
		239:i=22;
		240:i=44;
		241:i=88;
		242:i=176;
		243:i=125;
		244:i=250;
		245:i=233;
		246:i=207;
		247:i=131;
		248:i=27;
		249:i=54;
		250:i=108;
		251:i=216;
		252:i=173;
		253:i=71;
		254:i=142;
		255:i=1;
	endcase
end