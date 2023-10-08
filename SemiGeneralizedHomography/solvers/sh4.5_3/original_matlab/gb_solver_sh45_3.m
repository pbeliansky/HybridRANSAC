function sols_data = gb_solver(data)
[C0,C1] = setup_elimination_template(data);
C1 = C0 \ C1;
RR = [-C1(end-3:end,:);eye(12)];
AM_ind = [13,10,9,1,2,11,12,3,14,15,16,4];
AM = RR(AM_ind,:);
[V,D] = eig(AM);
V = V ./ (ones(size(V,1),1)*V(1,:));
sols(1,:) = V(2,:);
sols(2,:) = diag(D).';
sols_data = sols;

% Action =  y
% Quotient ring basis (V) = 1,x,x^2,x^3,x^2*y,x*y,x*y^2,x*y^3,y,y^2,y^3,y^4,
% Available monomials (RR*V) = x^3*y,x^2*y^2,x*y^4,y^5,1,x,x^2,x^3,x^2*y,x*y,x*y^2,x*y^3,y,y^2,y^3,y^4,
function [coeffs] = compute_coeffs(data)
coeffs(1) = data(6);
coeffs(2) = data(11);
coeffs(3) = data(16);
coeffs(4) = data(21);
coeffs(5) = data(26);
coeffs(6) = data(31);
coeffs(7) = data(36);
coeffs(8) = data(41);
coeffs(9) = data(46);
coeffs(10) = data(51);
coeffs(11) = data(56);
coeffs(12) = data(61);
coeffs(13) = data(66);
coeffs(14) = data(71);
coeffs(15) = data(76);
coeffs(16) = data(81);
coeffs(17) = data(86);
coeffs(18) = data(7);
coeffs(19) = data(12);
coeffs(20) = data(17);
coeffs(21) = data(22);
coeffs(22) = data(27);
coeffs(23) = data(32);
coeffs(24) = data(37);
coeffs(25) = data(42);
coeffs(26) = data(47);
coeffs(27) = data(52);
coeffs(28) = data(57);
coeffs(29) = data(62);
coeffs(30) = data(67);
coeffs(31) = data(72);
coeffs(32) = data(77);
coeffs(33) = data(82);
coeffs(34) = data(87);
coeffs(35) = data(92);
coeffs(36) = data(97);
coeffs(37) = data(8);
coeffs(38) = data(13);
coeffs(39) = data(18);
coeffs(40) = data(23);
coeffs(41) = data(28);
coeffs(42) = data(33);
coeffs(43) = data(38);
coeffs(44) = data(43);
coeffs(45) = data(48);
coeffs(46) = data(53);
coeffs(47) = data(58);
coeffs(48) = data(63);
coeffs(49) = data(68);
coeffs(50) = data(73);
coeffs(51) = data(78);
coeffs(52) = data(83);
coeffs(53) = data(88);
coeffs(54) = data(93);
coeffs(55) = data(98);
coeffs(56) = data(103);
coeffs(57) = data(4);
coeffs(58) = data(9);
coeffs(59) = data(14);
coeffs(60) = data(19);
coeffs(61) = data(24);
coeffs(62) = data(29);
coeffs(63) = data(34);
coeffs(64) = data(39);
coeffs(65) = data(44);
coeffs(66) = data(49);
coeffs(67) = data(54);
coeffs(68) = data(59);
coeffs(69) = data(64);
coeffs(70) = data(69);
coeffs(71) = data(74);
coeffs(72) = data(79);
coeffs(73) = data(84);
coeffs(74) = data(89);
coeffs(75) = data(94);
coeffs(76) = data(99);
coeffs(77) = data(5);
coeffs(78) = data(10);
coeffs(79) = data(15);
coeffs(80) = data(20);
coeffs(81) = data(25);
coeffs(82) = data(30);
coeffs(83) = data(35);
coeffs(84) = data(40);
coeffs(85) = data(45);
coeffs(86) = data(50);
coeffs(87) = data(55);
coeffs(88) = data(60);
coeffs(89) = data(65);
coeffs(90) = data(70);
coeffs(91) = data(75);
coeffs(92) = data(80);
coeffs(93) = data(85);
coeffs(94) = data(90);
coeffs(95) = data(95);
coeffs(96) = data(100);
coeffs(97) = data(105);
function [C0,C1] = setup_elimination_template(data)
[coeffs] = compute_coeffs(data);
coeffs0_ind = [1,57,77,2,1,18,58,37,78,3,2,19,59,38,79,4,3,20,60,39,80,6,1,57,77,7,6,23,63,2,18,1,42,58,37,78,83,8,7,24,64,3,19,2,43,59,38,79,84,9,...
8,25,65,4,20,3,44,60,39,80,85,10,9,26,66,5,21,4,45,61,40,81,86,6,1,57,77,11,7,23,6,63,2,37,42,58,18,1,78,83,12,11,28,68,8,24,7,47,64,3,...
38,43,59,19,2,79,84,88,13,12,29,69,9,25,8,48,65,4,39,44,60,20,3,80,85,89,14,13,30,70,10,26,9,49,66,5,40,45,61,21,4,81,86,90,6,57,77,11,7,42,...
63,23,58,18,1,6,37,78,83,15,12,28,11,68,8,43,47,64,24,59,19,2,7,38,79,84,88,16,15,32,72,13,29,12,51,69,9,44,48,65,25,60,20,3,8,39,80,85,89,92,...
11,63,23,6,42,83,15,12,47,68,28,64,24,7,11,43,84,88,16,32,15,72,13,48,51,69,29,65,25,8,12,44,85,89,92,17,16,33,73,14,30,13,52,70,10,45,49,66,26,61,...
21,4,9,40,81,86,90,93,17,34,74,31,14,53,71,46,50,67,27,62,22,5,10,41,82,87,91,94];
coeffs1_ind = [56,97,75,35,54,95,72,32,15,51,92,15,68,28,11,47,88,16,51,72,32,69,29,12,15,48,89,92,54,75,35,73,33,16,52,93,95,35,75,17,52,54,73,33,70,30,13,16,49,90,...
93,95,35,75,17,33,16,54,73,14,49,52,70,30,66,26,9,13,45,86,90,93,95,56,76,36,55,96,97,55,56,76,36,74,34,17,53,94,96,97,36,56,76,53,55,74,34,71,31,14,...
17,50,91,94,96,97,36,76,34,17,55,74,50,53,71,31,67,27,10,14,46,87,91,94,96];
C0_ind = [1,4,23,24,25,26,27,31,46,47,48,49,50,54,69,70,71,72,73,77,92,93,97,101,114,116,117,118,119,120,121,122,123,124,127,137,138,139,140,141,142,143,144,145,146,147,150,160,161,162,...
163,164,165,166,167,168,169,170,173,183,184,185,186,187,188,189,190,191,192,193,196,206,207,212,217,220,228,231,235,236,237,239,240,241,242,243,244,248,251,252,254,255,256,257,258,259,260,261,262,263,...
264,265,266,267,271,274,275,276,277,278,279,280,281,282,283,284,285,286,287,288,289,290,294,297,298,299,300,301,302,303,304,305,306,307,308,309,310,311,312,313,317,320,321,322,332,337,342,350,355,356,...
358,359,360,361,362,363,364,365,366,369,373,374,375,377,378,379,380,381,382,383,384,385,386,387,388,389,390,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,...
424,429,430,431,433,434,442,447,448,450,451,452,453,454,455,456,457,458,465,466,467,469,470,471,472,473,474,475,476,477,478,479,480,481,482,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,...
499,500,501,502,503,504,505,506,508,509,510,512,513,514,515,517,518,519,520,521,522,523,524,525,526,527,528,529];
C1_ind = [19,20,38,39,42,43,61,62,63,65,66,79,84,85,86,88,89,102,103,105,106,107,108,109,110,111,112,113,126,128,129,130,131,132,134,135,136,144,147,148,149,150,151,152,153,154,155,156,157,158,...
159,160,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,195,199,200,203,204,205,218,219,220,221,222,223,224,226,227,228,229,236,238,239,241,242,243,244,245,246,247,...
248,249,250,251,252,253,256,257,259,260,261,262,264,265,266,267,268,269,270,271,272,273,274,275,276];
C0 = zeros(23,23);
C1 = zeros(23,12);
C0(C0_ind) = coeffs(coeffs0_ind);
C1(C1_ind) = coeffs(coeffs1_ind);

