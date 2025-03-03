dtmc // Maybe make it continuous instead

const double max_speed = 0.26;

const double saf_threshold = 0.15;
const double saf_prob_good = 1 - (speed / max_speed);    // will be learned in future
const double saf_prob_bad = speed / max_speed;           // will be learned in future

const double pow_threshold = 4.0;
const double pow_prob_good = 1 - (speed / max_speed);    // will be learned in future
const double pow_prob_bad = speed / max_speed;           // will be learned in future

const double mov_threshold = 0.4;
const double mov_prob_good = speed / max_speed;          // will be learned in future
const double mov_prob_bad = 1 - (speed / max_speed);     // will be learned in future

module saf_qr
    s1: [0..1] init safety > saf_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s1=0 -> saf_prob_good : (s1'=1) + (1-saf_prob_good) : (s1'=0);
    [] s1=1 -> saf_prob_bad : (s1'=0) + (1-saf_prob_bad) : (s1'=1);
endmodule

module pow_qr
    s2: [0..1] init power < pow_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s2=0 -> pow_prob_good: (s2'=1) + (1-pow_prob_good) : (s2'=0);
    [] s2=1 -> pow_prob_bad : (s2'=0) + (1-pow_prob_bad) : (s2'=1);
endmodule

module mov_qr
    s3: [0..1] init move > mov_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s3=0 -> mov_prob_good: (s3'=1) + (1-mov_prob_good) : (s3'=0);
    [] s3=1 -> mov_prob_bad : (s3'=0) + (1-mov_prob_bad) : (s3'=1);
endmodule


// rewards
//     [safety_qr_met] true : 1;
//     [safety_qr_not_met] true : -1;
// endrewards

const double safety = 0.05534873902797699;
const double power = 37.93861389160156;
const double move = 0.3914274275302887;
const double speed = 0.26;
