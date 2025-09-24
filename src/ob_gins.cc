/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/common/earth.h"
#include "src/common/types.h"

#include "src/fileio/filesaver.h"
#include "src/fileio/gnssfileloader.h"
#include "src/fileio/imufileloader.h"

#include "src/factors/gnss_factor.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_manifold.h"
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"
// 缂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柛顐ｆ礀缁愭鈧箍鍎卞ú銊╁础濮樿京妫柣妤€鐗婇幊涔?闂傚倷绀侀幖顐λ囬柆宥呯；闁绘梻鍘х紒鈺傘亜閵忋垻鍩€闂傚倸鍊搁崐椋庢閿熺姴纾婚柛鏇ㄥ灠绾惧綊鏌涜椤ㄥ懘宕掗妸鈺傜厪闁割偅绻冮惌妤佺箾瀹€濠侀偗闁哄本绋戦悾婵堚偓锝庝憾濞差厾绱撴担鎻掍壕闂佸憡娲﹂崹閬嶅煕閹烘鐓曢柍鈺佸枤濞堟洘銇勯妷锔炬噰闁哄被鍊濆畷褰掝敃閵忣澀绱濋柣搴ゎ潐濞叉ê鐣濈粙璺ㄦ殾闁告鍊ｉ悢铏圭當闁告繂瀚崰濠囨⒒?#include "src/bridge/imu_adapter.h"
// Wheel-IMU state for wheel-specific output
#include "src/wheel/integration_state_wheel.h"
// (Wheel preintegration headers were included temporarily for wiring; removed per request.)

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);
// Wheel-only writer to allow saving dual navigation results later
void writeNavResultWheel(double time, const Vector3d &origin, const WheelIntegrationState &state, FileSaver &navfile,
                         FileSaver &errfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";

    auto ts = absl::Now();

    // 闂傚倸鍊峰ù鍥х暦閸偅鍙忛柡澶嬪殮濞差亜鐓涢柛婊€鐒﹂弲顏堟偡濠婂嫬鐏村┑锛勬暬楠炲洭寮剁捄銊モ偓鐐差渻閵堝棗绗傞柤鍐茬埣瀹曘垽骞囬悧鍫濃偓鍨殽閻愯尙浠㈤柛鏃€纰嶇换娑氫沪閸屾艾顫囬悗?
    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢敃鈧悿顔姐亜閹板爼妾柛瀣樀閺岋綁骞橀崘宸敨濠电偞鍨崹鍦不娴兼潙绠归弶鍫濆⒔瀹€娑㈡煛閳ь剟骞庨懞銉㈡嫼闂佸搫鍊堕崕鎻掆枍閸涘瓨鐓曢柣鏂垮级濞呭洦淇?
    // processing time
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // 闂傚倸鍊风粈渚€骞栭位鍥敃閿曗偓閻ら箖鏌嶉崫鍕櫣闁稿被鍔嶇换娑橆啅椤旇崵鐩庨梺鍏肩摃椤濡甸崟顖氱闁糕剝銇炴竟鏇㈡⒒娴ｅ摜鏋冩い鏇嗗洦鐓€闁挎繂顦弰銉╂煕閺囥劌鐏犵紒鈧崘顔界厓闁告繂瀚禍鐐烘煕閻旀彃浜?
    // number of iterations
    int num_iterations = config["num_iterations"].as<int>();

    // 闂傚倸鍊风粈渚€骞栭位鍥敃閿曗偓閻ょ偓绻濇繝鍌涘櫧闁活厽鐟╅弻鈥愁吋鎼粹€崇闂侀€炲苯鍘哥紒鑸靛哺閻涱噣宕奸妷銉ь唽濠德板€楃粊宕囩磽閸屾艾鈧悂宕愭搴㈩偨闁跨喓濮寸粣妤佷繆閵堝懏鍣烽柍褜鍓欓崯鏉戠暦閵娾晩鏁囬柛銉㈡櫅閺嬫垹绱掔紒妯肩疄鐎规洘鍎奸ˇ瀛樸亜閿旇寮慨濠冩そ瀹曨偊宕熼锝嗩唲闂備胶绮〃鍛存晝椤忓牆绠?
    // Do GNSS outlier culling
    bool is_outlier_culling = config["is_outlier_culling"].as<bool>();

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁嶉崟顒佹濠德板€曢崯顖氱暦閺屻儲鐓曠€光偓閳ь剟宕曢幋鐘电闁哄稁鍘介悡娆撴煟濡も偓閻楀﹦娆㈤懠顒傜＜闁逞屽墮閻ｆ繈宕熼鍌氬箰闁诲骸绠嶉崕杈╂崲閹烘梹顫曢柟娈垮枤绾惧ジ鎮归崶褍绾фい銉ｅ灲閺?    // initialization
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // 闂傚倸鍊搁崐宄懊归崶褜娴栭柕濞炬櫆閸ゅ嫰鏌ょ粙璺ㄤ粵婵炲懐濮垫穱濠囧Χ閸屾矮澹曢梻浣风串缁蹭粙鎮樺杈╃當闁绘梻鍘ч悞鍨亜閹哄棗浜惧銈嗘穿缂嶄線銆佸Δ鍛妞ゆ劑鍊曢埀顒傚仱濮婃椽骞愭惔鈶╂嫽闂佺儵鍓濆Λ鍐ㄧ暦?
    // data file
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string imupath;
    int imudatalen  = 0;
    int imudatarate = 0;
    std::string outputpath = config["outputpath"].as<std::string>();

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢敃鈧壕褰掓偡濞嗗繐顏柣婵嗙埣閺屾盯鍩勯崗鈺傚灥閳诲秹鎮╅崗鍛畾闂侀潧鐗嗙€氼噣藝閿斿墽纾奸柛鎾茬娴狅妇绱掔紒妯兼创鐎规洖宕埢搴ㄥ箣濞嗗苯浜鹃柛锔界叀濮婂宕掑顑藉亾閻戣姤鍊块柨鏇楀亾妞ゎ厼鐏濊灒闁兼祴鏅濋悡瀣⒑閸撴彃浜濇繛鍙夛耿瀹曟垿顢旈崼鐔哄幈闂佹枼鏅涢崯浼村煀閺囥垺鐓涢柛婊€绀佹晶鎾煛鐏炲墽娲存い銏℃礋閺佹劙宕堕埡鍐ㄥ笓闂傚倷绀侀幗婊勬叏閻㈠憡鏅濋柕蹇嬪€曢拑鐔哥箾閹寸偟鐭岄柣銈傚亾闂備浇顫夊畷妯衡枖濞戭潿鈧倿宕烽鐘碉紳婵炴挻鑹惧ú銊╁储濞戙垺鐓曢柟鎹愭硾閺嬪孩銇勯弴顏嗙М妞ゃ垺鐟ラ埢搴ㄦ倷椤戭偀鏅犲缁樻媴閸濆嫬濮︽繝娈垮枛缁夌懓鐣峰鈧弫鎰板幢濞嗗海鎸夌紓鍌氬€烽懗鍫曞磻閹炬剚鐔嗘慨妞诲亾閽樻繈寮堕崼姘珖闁?
    bool use_main = false, use_wheel_left = false, use_wheel_right = false;
    if (config["run_mode"]) {
        auto rm = config["run_mode"];
        if (rm["imu_main_enable"])    use_main       = rm["imu_main_enable"].as<bool>();
        if (rm["wheel_left_enable"])  use_wheel_left = rm["wheel_left_enable"].as<bool>();
        if (rm["wheel_right_enable"]) use_wheel_right= rm["wheel_right_enable"].as<bool>();
    }

    // 闂傚倸鍊峰ù鍥х暦閸偅鍙忛柡澶嬪殮濞差亜鐓涢柛婊€鐒﹂弲顏堟偡濠婂嫬鐏村┑锛勬暬楠炲洭寮剁捄銊モ偓鐐差渻閵堝棗鍧婇柛瀣尰娣囧﹪顢曢敐蹇氣偓鍧楁煛瀹€瀣埌閾绘牠鏌嶈閸撶喖骞冭椤劑宕煎┑濠傜厴?IMU 闂傚倸鍊搁崐鐑芥嚄閸撲焦鍏滈柛顐ｆ礀閻ょ偓绻濋棃娑卞剰缁炬儳娼￠弻鏇㈠醇濠靛洤鐝旈梺鍏兼緲閻忔繈婀侀梺缁樻尭濞寸兘骞楅悩缁樼厽闁圭虎鍨版禍楣冩⒑鐠囨煡顎楃紒鐘茬Ч瀹曟洟宕￠悘缁樻そ婵℃悂鍩℃担绋挎闂備胶顭堥惉濂稿磻閻愮儤鍋傞柕澶涘缁犻箖鏌熺€电鍓遍柣鎺曟椤儻顧傜紓宥勭窔楠炲啫螖閳ь剟锝炲┑瀣ㄩ柕澹倻妫紓鍌氬€风粈渚€顢栭崨姝ゅ洭顢涢悜鍡樻櫍婵犻潧鍊婚…鍫濐啅濠靛洢浜滈柡宥冨妿閳洟鏌涢悩宕囧⒌婵﹦绮粭鐔煎焵椤掆偓椤洭鎳￠妶鍡楊伕闂佺粯姊婚崢褏绮堟径灞稿亾閸忓浜鹃梺鍛婃处閸嬪嫰鎮楅鐑嗘富闁靛牆妫涙晶閬嶆煕鐎ｎ偆娲撮柟顔界懄缁绘繈宕堕妸褍骞楅梻浣哥秺閸嬪﹪宕归鍕劦妞ゆ垼妫勬禍楣冩⒒娴ｄ警鐒炬い鎴濇楠炴垿宕堕鈧弰銉╂煃瑜滈崜姘跺Φ閸曨垰绠抽柟瀛樼箥娴犺偐绱撴担鎻掍壕闂佺硶鍓濈粙鎺楁偂閻旈晲绻嗛柕鍫濆€告禍楣冩⒑缂佹﹩娈旈柨鏇ㄤ邯瀵偄顓奸崶锝呬壕闁挎繂楠搁弸鐔兼煕婵犲倻浠涚紒缁樼洴楠炲鎮欓崹顐㈡婵犵數鍋涢悧濠偯哄鈧俊鐢稿礋椤栨碍顥濋柣鐘充航閸斿酣宕濋鐐粹拺?
    YAML::Node imu_node;
    if (use_main && config["imu_main"]) {
        imu_node = config["imu_main"];
    } else if (use_wheel_left && config["imu_wheel_left"]) {
        imu_node = config["imu_wheel_left"];
    } else if (use_wheel_right && config["imu_wheel_right"]) {
        imu_node = config["imu_wheel_right"];
    }

    if (imu_node) {
        if (imu_node["file"])      imupath     = imu_node["file"].as<std::string>();
        if (imu_node["columns"])   imudatalen  = imu_node["columns"].as<int>();
        if (imu_node["rate_hz"])   imudatarate = imu_node["rate_hz"].as<int>();
        // 闂傚倸鍊搁崐鐑芥嚄閸洍鈧箓宕奸姀鈥冲簥闂佸湱鍎ら〃鍛村磼閵娧勫枑闁哄啫鐗勯埀顑跨閳诲酣骞樺畷鍥╂澑闂備礁鎼ˇ鍐测枖閺囥垺鍎撻柛鏇ㄥ灡閸嬧剝绻濇繝鍌氭殶缂佺姵鐓￠弻锟犲川閻楀牏銆愰柧缁樼墵閺屾盯骞囬崜浣稿煂婵炲瓨绮庨崑鎾舵崲濞戙垹绠ｉ柣鎰仛閸ｈ棄鈹戦悙鑼婵炲弶锕㈡俊?
        // 闂傚倸鍊搁崐鎼佸磹閻戣姤鍤勯柛顐ｆ磵閳ь剨绠撳畷濂稿閳ュ啿绨ラ梻浣告贡閸庛倝銆冮崨鏉戠＜闁靛ě鍕瀾闂佸搫鍟悧鍕焵椤戣法顦﹂柍璇查叄楠炴ê鐣烽崶鈺傛緫闂傚倷绀侀幖顐λ囬崘娴嬫灃闁哄洢鍨洪崕鎾绘煕閺囥劌鐏￠柛瀣у墲缁绘繃绻濋崒姘缂備胶濮甸悡锟犲蓟閿熺姴宸濋柣妤€鐗忛妴鎰版煣缂佹澧甸柡灞剧洴楠炲洭宕滄担绋跨厒濠电偛鐡ㄧ划搴ㄥ磻閹捐埖宕叉繛鎴欏灩缁狅綁鏌ｉ幇顒備粵闁革綀妫勯埞鎴︽倷閼碱剙顤€濠电偛寮堕敃銏′繆?
        if (config["imufile"])     imupath     = config["imufile"].as<std::string>();
        if (config["imudatalen"])  imudatalen  = config["imudatalen"].as<int>();
        if (config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();
    }
    // Ensure output directory exists
    try {
        if (!outputpath.empty()) {
            std::filesystem::path outdir(outputpath);
            if (!std::filesystem::exists(outdir)) {
                std::filesystem::create_directories(outdir);
                std::cout << "[info] output path not found. Created: " << outdir.string() << std::endl;
            }
        }
    } catch (const std::exception &e) {
        std::cout << "[error] failed to create output path: " << outputpath << ", reason: " << e.what() << std::endl;
        // Continue; subsequent file open checks will handle failure
    }
    // 闂傚倸鍊搁崐椋庣矆娓氣偓瀹曘儳鈧綆鍠楅崕鎴犳喐閻楀牆绗掔痪鎯ф健閺岀喓绱掑Ο娲殝闂佽偐澧楃€笛囧Φ閸曨垰绠婚柛妤冨仧娴犲ジ姊洪崷顓熷殌闁绘牕銈稿濠氬即閻旇櫣鐦堥棅顐㈡处缁嬫帡顢撳澶嬧拺缂備焦蓱鐏忋劑鏌涚€ｎ偅宕屾慨濠傤煼瀹曟帒鈻庨幋婵嗩瀴婵犵數濮伴崹娲€﹂崶鈺佸灊闁哄啫鐗嗛柋鍥煛閸モ晛鏋庡ù鐘荤畺濮婅櫣鍖栭弴鐐测拤濡炪値鍘煎ú顓㈠春閵忋倕绫嶉柍褜鍓熸俊鐢稿礋椤栨氨顔撻梺鍛婃处閸橀箖鏁嶉悢鍏尖拺闂傚牊绋撴晶娑㈡煙閸涘﹤鈻曠€殿喛顕ч埥澶娾枎閹寸姷妲囬梻浣告啞濞插秹寮笟鈧畷鎴﹀箻閸ㄦ稑浜鹃柨婵嗛婢ь垱銇勯锝嗙缂佺粯绻堝Λ鍐ㄢ槈濮橆剦鏉搁梻浣侯焾缁绘垹鍒掑澶娢﹂柛鏇ㄥ灠缁秹鏌涢妷鎴濆暞閸庮亞绱撻崒娆掑厡濠殿喗鍎宠灋闁告洦鍨辩粻鎺楁⒒娴ｈ櫣甯涙い顓炵墢娴滅鈻庨幇顔尖叞婵犵數濮烽弫鍛婃叏椤撱垹纾婚柟鍓у仺閳ь剚甯掗～婵嬫晲閸涱剙顥氶梺璇插椤旀牠宕伴弽顓熷€舵慨姗嗗墻閸ゆ鏌涢弴銊ュ箰闁稿鎹囬弫鎰償閳ヨ尙鍑归梻浣虹帛缁洪箖宕滃┑瀣疄闁靛ň鏅涚粻鎶芥煙閹呬邯濠殿喖娲ら—鍐Χ鎼粹€崇闂佺粯顨嗛幑鍥Υ娴ｈ倽鐔兼嚒閵堝洨妲囨繝娈垮枟閿曗晠宕滃☉銏℃櫖?    if (imudatalen == 0 && config["imudatalen"])  imudatalen  = config["imudatalen"].as<int>();
    if (imudatarate == 0 && config["imudatarate"]) imudatarate = config["imudatarate"].as<int>();

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢妶鍌氫壕婵ê宕崢瀵糕偓瑙勬礉椤鈧潧銈稿鍫曞箣閻樺灚姣庢繝鐢靛仦閹稿宕洪崘顔肩；闁瑰墽绮悡娆撴煣韫囷絽浜濈€规洖鐬奸埀顒冾潐濞叉﹢鏁冮姀銈冣偓浣割潨閳ь剟骞冮妶鍡樺闁圭粯甯婃竟鏇炩攽椤旂瓔鐒鹃柛鈺傜墵閹繝寮撮姀锛勫弳闂佸搫鍟ú锕偹夋径瀣ㄤ簻闁靛鍎洪崕鏃堟煛瀹€鈧崰鏍€佸▎鎾崇缁炬媽椴搁敍鍡涙⒒娴ｄ警鏀版繛鍛礋閹嫰顢涢悙顏佸亾娓氣偓瀵噣宕煎┑鍫濃偓鐐烘⒑闂堟丹娑欐媴闂€鎰梾
    // consider the Earth's rotation
    bool isearth = config["isearth"].as<bool>();

    GnssFileLoader gnssfile(gnsspath);
    // 闂傚倸鍊风粈渚€骞栭銈囩煋闁绘垶鏋荤紞鏍ь熆鐠虹尨鍔熼柡?run_mode 闂傚倸鍊风粈渚€骞栭銈囩煋闁哄鍤氬ú顏勭厸闁告粈鐒﹂弲?IMU 闂傚倸鍊峰ù鍥ь浖閵娾晜鍤勯柤绋跨仛濞呯姵淇婇妶鍌氫壕闂佷紮绲介悘姘辩箔閻旂厧鐒垫い鎺嗗亾妞ゆ洩缍佸畷濂稿即閻愬秲鍔戦弻銊╁棘閸喒鎸冮梺绯曟櫅缁夌懓顫忓ú顏勭閹艰揪绲哄Σ鍫ユ⒑缂佹﹩娈旈柟铏姍閸┾偓妞ゆ帒瀚☉褔鏌ｉ埥鍡橆€淯/闂傚倷绀侀幖顐λ囬柆宥呯？闁圭増婢樼粣妤呮煛瀹ュ骸浜濇い顐ｆ礃閵囧嫰鍨惧畷鍥煑U闂傚倸鍊烽悞锔锯偓绗涘懐鐭欓柟娆″眰鍔戦崺鈧い鎺戝€荤壕濂稿级閸稑濡跨紒鐘靛仧閳ь剝顫夊ú婊堝礂濡绻嗛柟闂磋閳ь剨绠撻獮瀣攽閸ヨ泛鏅ラ梻鍌氬€风粈渚€骞夐敓鐘冲仭闁挎洖鍊归崑瀣繆閵堝倸浜惧銈庝簻閸熸潙鐣烽幒妤佸€烽柤纰卞墻濡茬増绻濋悽闈涗粶婵☆偅顨堥幑銏ゅ箛椤掑娈?IMU::is_wheel
    bool imu_is_wheel = (use_wheel_left || use_wheel_right);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate, imu_is_wheel);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // 闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勭粻鐘绘煙閹冩闁搞儺鍓欑粻顕€鏌涢幘宕囦虎妞わ附澹嗛幑銏犫攽鐎ｎ亞鍊為悷婊冪箻閹繝骞嬮敂瑙ｆ嫼缂備礁顑嗙€笛冿耿閹殿喚纾奸悗锝庡亜缁楁帡鏌?
    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R;

    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();
    //婵犵數濮烽弫鎼佸磻濞戙埄鏁嬫い鎾跺枑閸欏繘鎮楅悽鐢点€婇柛瀣尭閳藉骞掗幘瀵稿綃闁诲孩顔栭崰姘卞垝椤栨粎绱﹂柣锝呯灱缁憋箓鏌嶉搹瑙勭oise 闂傚倸鍊峰ù鍥х暦閸偅鍙忛柡澶嬪殮濞差亜鐓涢柛婊€鐒﹂弲顏堟偡濠婂嫬鐏村┑锛勬暬楠炲洭寮剁捄銊モ偓鐐差渻閵堝棗绗傞柣鎺炵畱閳诲秹寮介鐔叉嫼闂佸憡鎸昏ぐ鍐╃濠靛牏纾奸悹鍥ㄥ絻椤忣厽顨ラ悙鎼疁闁诡喒鏅濈槐鎺懳熼悡搴＄疄闂傚倷鐒︾€笛囧蓟閵娾晛绠规い鎰剁导缁诲棗鈹戦崒姘暈闁绘挻娲橀妵鍕敇閻旈浠撮梺璇查獜缂嶄線寮婚悢鑲╁彄妞ゆ挾鍋涚粊顕€鎮楃憴鍕鐎规洦鍓熼崺銏℃償閵娿儳鐤€濡炪倖娲栧Λ娑㈡儗濡ゅ懏鈷掗柛灞剧懅閸斿秹鎮楃粭娑樻噽閻瑩鏌熷▎鈥崇湴閸旀垿宕洪埀顒併亜閹烘垵鈧崵澹曟總绋跨骇闁割偅绋戞俊璺ㄧ磼閻橀潧浠ч柍褜鍓濋～澶娒鸿箛娑樼？闁汇垻顭堥弸渚€鏌涢幇闈涙灈闁绘帒鐏氶妵鍕箳閹存繍浠奸梺鎼炲妼閸婂綊濡甸崟顖氬唨闁靛ě鈧慨鍥ㄧ節濞堝灝鐏￠柟鍛婂▕楠炲啳銇愰幒鎴犵暢闂佸湱鍎ら崹鐢糕€栨径鎰拺闁告繂瀚ˉ婊呯磼缂佹﹫鑰跨€殿喖顭烽弫鎾绘偐閼碱剦妲伴梻浣告啞濞诧箓宕戦崟顖氭辈闁绘柨鍚嬮悡娆撴煟閿濆懓瀚板瑙勎攗model
    if (imu_node && imu_node["imunoise"]) {
        auto ino = imu_node["imunoise"];
        parameters->gyr_arw      = (ino["arw"].as<double>()) * D2R / 60.0;     // deg/sqrt(hr) -> rad/s^0.5
        parameters->acc_vrw      = (ino["vrw"].as<double>()) / 60.0;           // m/s/sqrt(hr) -> m/s^1.5
        parameters->gyr_bias_std = (ino["gbstd"].as<double>()) * D2R / 3600.0; // deg/hr -> rad/s
        parameters->acc_bias_std = (ino["abstd"].as<double>()) * 1.0e-5;       // mGal -> m/s^2
        parameters->corr_time    = (ino["corrtime"].as<double>()) * 3600.0;    // hr -> s
        // 濠电姷鏁告慨鐢割敊閺嶎厼闂い鏍ㄧ矊缁躲倕螖閿濆懎鏆欓柦鍐枔閹叉瓕绠涢幘顖涚€婚梺闈涚箞閸婃洖鏁梻浣哥枃濡椼劑鎳楅崼鏇炵９闁绘劗鍎ら埛鎴犵磽娴ｇ櫢渚涢柣鎺楃畺閹绠涢弮鍌涘櫚濡?
        if (ino["gsstd"]) parameters->gyr_scale_std = ino["gsstd"].as<double>() * 1e-6; // ppm -> ratio
        if (ino["asstd"]) parameters->acc_scale_std = ino["asstd"].as<double>() * 1e-6; // ppm -> ratio
    } else {
        parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
        parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
        parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
        parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
        parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;
    }

    bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;

    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // 闂傚倸鍊搁崐宄懊归崶褜娴栭柕濞炬櫆閸ゅ嫰鏌ょ粙璺ㄤ粵婵炲懐濮垫穱濠囧Χ閸屾矮澹曢梻浣风串缁蹭粙鎮樺璺虹闁告侗鍨遍崑姗€鏌嶉妷銉ュ笭濠㈣娲熷濠氬磼濮樺崬顤€缂備礁顑嗛幐鍓у垝閺冨牊鎯為柛锔诲幘閿?
    // data alignment
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur;
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime);

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁嶉崟顒佹濠德板€曢崯顖氱暦閺屻儲鐓曠€光偓閳ь剟宕曢幋鐘电闁哄稁鍘介悡娆撴煟濡も偓閻楀﹦娆㈤懠顒傜＜闁逞屽墮閻ｆ繈宕熼鍌氬箰闁诲骸绠嶉崕杈╂崲閹拌埇鈧懘顢楅崒婊咃紲婵犮垼娉涢…顒勫吹閻旇櫣纾奸弶鍫涘妽瀹曞瞼鈧娲橀敃銏ょ嵁閸℃凹妲奸梺缁樼箥閸ㄥ磭妲愰幘瀛樺濞寸姴顑呴幗鐢电磽娴ｇ瓔鍤欓悗姘嵆閻涱喛绠涢弴妤€浜鹃柨婵嗛閺嬬喖鏌￠崪浣稿⒋闁哄本鐩鎾Ω閵夈儳顔戞繝娈垮櫙缁蹭粙鎮ч幘鎰佹綎缂備焦蓱婵挳鏌ｉ悢绋款棆缂佷線顥撶槐鎾存媴閸撳弶笑闂佸鏉垮妤?
    Vector3d station_origin = gnss.blh;
    parameters->gravity     = Earth::gravity(gnss.blh);
    gnss.blh                = Earth::global2local(station_origin, gnss.blh);

    // 缂傚倸鍊搁崐鎼佸磹閻戣姤鍊块柨鏇楀亾妞ゎ偄绻楅妵鎰板箳閹惧厖鐢婚梻渚€娼ч敍蹇涘川椤栨凹妲辨繝寰锋澘鈧呭緤娴犲鐤い鎰堕檮閻撱儵鏌曢崼婵愭Ч闁绘挻鐟╁娲敇閵娧呮殸濡ょ姷鍋為敃銏ゅ蓟濞戙垹惟闁靛鍎板Ч妤冪磽娴ｄ粙鍝洪悽顖ょ節閵嗕礁鈽夊Ο閿嬫杸闂佸綊鍋婇崜姘额敊閸曨垱鈷戞慨鐟版搐閻掓椽鏌涢妸銉ｅ仮鐎规洘鍨剁换婵嬪炊瑜忛悾?    parameters->station = station_origin;

    // Wheel preintegration wiring was removed per request; proceed with main IMU chain.

    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::deque<std::shared_ptr<Adapter::UnifiedPreintegrator>> preintegrationlist;
    std::deque<GNSS> gnsslist;
    std::deque<double> timelist;

    Adapter::UnifiedPreintegrator::Options preintegration_options = Adapter::GetOptions(isuseodo, isearth);

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁嶉崟顒佹濠德板€曢崯顖氱暦閺屻儲鐓曠€光偓閳ь剟宕曢幋鐘电闁哄稁鍘介悡娆撴煙濞堝灝鏋涙い锝呫偢閺屾稓鈧綆鍋勬慨宥夋煛鐏炶鈧繂鐣烽锕€唯妞ゆ棁濮ら惁婊堟⒒?    // initialization
    IntegrationState state_curr = {
        .time = round(gnss.time),
        .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = initvel,
        .bg   = initbg,
        .ba   = initba,
        .sodo = 0.0,
        .abv  = {bodyangle[1], bodyangle[2]},
    };
    std::cout << "Initilization at " << gnss.time << " s " << std::endl;

    statelist[0]     = state_curr;
    statedatalist[0] = Adapter::StateToData(state_curr, preintegration_options);
    gnsslist.push_back(gnss);

    double sow = round(gnss.time);
    timelist.push_back(sow);

    // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁嶉崟顒佹濠德板€曢崯顖氱暦閺屻儲鐓曠€光偓閳ь剟宕曢幋鐘电闁哄稁鍘介悡娆撴煟濡も偓閻楀﹦娆㈤懠顒傜＜闁逞屽墮閻ｆ繈宕熼鍌氬箰闁诲骸绠嶉崕杈殽缁嬫鍤曞Δ锝呭暞閻撴盯鏌涢埄鍏╂垹绮堥崼銏㈢＜閺夊牄鍔岀粭姘扁偓鍨緲鐎氼剝鐏掗梺鎯х箻閳ь剚绋戦幗瀣⒒?
    // Initial preintegration
    preintegrationlist.emplace_back(
        Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr, preintegration_options, imu_pre.is_wheel));

    // 闂傚倸鍊峰ù鍥х暦閸偅鍙忛柡澶嬪殮濞差亜鐓涢柛婊€鐒﹂弲顏堟偡濠婂嫬鐏村┑锛勬暬楠炲洭寮剁捄銊モ偓鐐烘⒑闂堟丹娑㈠焵椤掑嫬鍑犻柡宥庡幗閻撶喖骞栭幖顓炵仯鐎光偓濞戙垺鐓涢柛娑卞枤缁犵偞銇勯姀鈩冾棃闁哄苯妫楅濂稿幢濞嗗繐绠?GNSS
    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh);
    gnss.blh            = Earth::global2local(station_origin, gnss.blh);

    // 闂傚倷绀侀幖顐λ囬鐐村亱闁糕剝顨愰懓鍧楀级閸碍娅嗘い顐ｆ礃閵囧嫰骞橀崡鐐典患缂備讲鍋撶€光偓閸曨剛鍘搁悗骞垮劚缁绘帞妲愰幍顔剧＜閻犲洤寮堕ˉ銏ゆ煛?
    std::shared_ptr<MarginalizationInfo> last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;

// 濠电姷鏁搁崑鐐哄垂閸洖绠伴柟闂寸贰閺佸嫰鏌涢弴銊ュ箻闁告宀搁幃妤€鈽夊▎妯煎姺闂佸磭绮褰掑Φ閸曨喚鐤€闁圭偓娼欏▍銈咁渻閵囶垯绀佸ú锕傚煕閹烘鐓曢悘鐐插⒔閻倖淇婇銏ゎ€楅柍瑙勫灴閸┿儵宕卞Δ鈧猾宥夋⒑?
    sow += INTEGRATION_LENGTH;

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) {
            break;
        }

        // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁缂傚倷鐒﹁摫闁告瑥绻橀弻鐔虹磼閵忕姵鐏堥梺?IMU 闂傚倸鍊搁崐宄懊归崶褜娴栭柕濞炬櫆閸ゅ嫰鏌ょ粙璺ㄤ粵婵炲懐濮垫穱濠囧Χ閸屾矮澹曢梻?
        // Add new imu data to preintegration (闂傚倸鍊搁崐椋庢閿熺姴纾婚柛鏇ㄥ灠绾惧綊鏌涜椤ㄥ懘宕掗妸鈺傜厪闁割偅绻冮惌妤佺箾?
        preintegrationlist.back()->addNewImu(imu_cur);

        imu_pre = imu_cur;
        imu_cur = imufile.next();

        if (imu_cur.time > sow) {
            // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁缂傚倷鐒﹁摫闁告瑥绻橀弻鐔虹磼閵忕姵鐏堥梺?GNSS 濠电姴鐥夐弶搴撳亾濡や焦鍙忛柣鎴ｆ绾惧鏌ｉ幇顒佹儓缁炬儳鐏濋埞鎴﹀磼濮橆剦妫岄梺杞扮椤戝懘婀侀梺绋跨箰閸氬绱為幋锔界厽妞ゆ挾鍣ュ▓婊堟煛鐏炵硶鍋撻幇浣告倯闂佸憡鍔戦崝宀勨€栨径鎰拺缂備焦蓱鐏忋劑鏌涚€ｎ偅宕屾慨?GNSS
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) {
                gnsslist.push_back(gnss);

                gnss = gnssfile.next();
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold)) {
                    gnss = gnssfile.next();
                }

                // GNSS 婵犵數濮烽弫鎼佸磻閻愬搫鍨傞柛顐ｆ礀缁犲綊鏌嶉崫鍕櫣闁稿被鍔岄湁闁绘ê妯婇崕蹇涙煢閸愵亜鏋涢柡灞剧洴婵＄兘顢欓悡搴浇闂備胶绮幐鎾磻閹剧粯鐓熼幖杈剧磿閻ｎ參鏌涙惔鈥宠埞閻撱倝鏌曟繛鐐珔闁?
                // do GNSS outage
                if (isuseoutage) {
                    if (lround(gnss.time) == outagetime) {
                        std::cout << "GNSS outage at " << outagetime << " s" << std::endl;
                        for (int k = 0; k < outagelen; k++) {
                            gnss = gnssfile.next();
                        }
                        outagetime += outageperiod;
                    }
                }

                parameters->gravity = Earth::gravity(gnss.blh);
                gnss.blh            = Earth::global2local(station_origin, gnss.blh);
                if (gnssfile.isEof()) {
                    gnss.time = 0;
                }
            }

            // IMU 闂傚倸鍊搁崐椋庣矆娴ｉ潻鑰块弶鍫氭櫅閸ㄦ繃銇勯弽銊х煁闁哄棙绮撻弻鐔兼倻濮楀棙鐣堕梺缁樺笒閻忔岸濡甸崟顖氱闁瑰瓨绺鹃崑鎾诲传閵夛附娈伴梺鐓庢憸閸嬶絾绂嶅鍫熺叆闁哄啫娴傚鎰箾閸涱叏韬柡?            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) {
            } else if (isneed == 1) {
                preintegrationlist.back()->addNewImu(imu_cur);

                imu_pre = imu_cur;
                imu_cur = imufile.next();
            } else if (isneed == 2) {
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }

            // 婵犵數濮烽弫鎼佸磻閻愬搫鍨傞柛顐ｆ礀缁犱即鏌熼梻瀵歌窗闁轰礁瀚伴弻娑㈠即閵娿儱绠婚梺鍛婎殕瀹€鎼佸箖濡も偓閳藉鈻庡Ο鐓庡Ш闂備礁纾划顖氼潖瑜版帒桅闁告洦鍠氶悿鈧梺鍦亾濞兼瑥鈻嶉妶鍜佹富闁靛浂鍨粈浣该洪敃鍌氱厱闁圭儤顨嗛悡鏇㈡倶閻愭彃鈷旈柣顓滃€栨穱濠囶敃閵忋値鈧鏌嶇憴鍕伌闁糕斂鍎靛畷鍗炍旈埀顒傜尵瀹ュ鈷?            // next time node
            timelist.push_back(sow);
            sow += INTEGRATION_LENGTH;

            // 闂傚倷娴囧畷鐢稿窗閹邦喖鍨濋幖娣灪濞呯姵淇婇妶鍛櫣缂佺姳鍗抽弻娑樷槈濮楀牊鏁惧┑鐐叉噽婵炩偓闁哄矉绲借灒婵炲棙鍎冲▓顓犵磽娓氬洤浜滅紒澶婄秺楠炲啳銇愰幒鎴滅炊闂佸憡娲﹂崜姘跺磿閹剧粯鈷戠紓浣股戠亸鎵磼鐠囪尙澧︾€殿喖顭烽弫鎰板川閸屾粌鏋涚€规洖缍婇、娆撳箚瑜嶇紓姘舵⒒閸屾瑧绐旈柍褜鍓涢崑娑㈡嚐椤栫偛鍌ㄩ柛婵勫劤绾惧ジ鏌嶈閸撴岸骞忛崨顖涘枂闁告洦鍋勬导搴ㄦ⒒娴ｈ櫣甯涢柛鏃€娲熼獮鏍敃閵堝洨鐒奸梺绋跨灱閸嬬偤鎮¤箛鎿冪唵閻犻缚娅ｆ晶鏇㈡煕閺冣偓瀹€鎼佸蓟閿濆绠婚柛鎰ゴ閸嬫捇宕烽鐔峰簥濠电娀娼уú銊у姬閳ь剟姊洪崨濠傚闁告柨瀛╃粋鎺撶附閸涘ň鎷?
            state_curr                               = preintegrationlist.back()->currentStateMain();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Adapter::StateToData(state_curr, preintegration_options);

            // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢敂缁樻櫈闂佸憡渚楅崹顏堝磻閹炬剚娼╅柣鎾抽椤偆绱撴担浠嬪摵闁圭懓娲悰顔碱潨閳ь剙顕ｉ崼鏇炵妞ゆ挾鍋為鎾斥攽閻樻鏆俊鎻掓嚇瀹曡瀵奸弶鎴狀啇濡炪倖鍔х€靛矂寮€ｎ剚鍠愰柡鍌涱儥濞兼牕鈹戦悩瀹犲缁炬儳銈搁弻鏇㈠醇濠靛洤娅ら梺缁樼箖缁诲倿鈥?
            // construct optimization problem
            {
                ceres::Problem::Options problem_options;
                problem_options.enable_fast_removal = true;

                ceres::Problem problem(problem_options);
                ceres::Solver solver;
                ceres::Solver::Summary summary;
                ceres::Solver::Options options;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
                options.num_threads                = 4;

                // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩鍐蹭画闂侀潧顦弲娑氬閸︻厽鍠愰柣妤€鐗嗙粭鎺撴叏鐟欏嫮鍙€闁哄矉缍佸顕€宕掑顑跨帛闂?                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    // 婵犵數濮烽弫鎼佸磻閻樿绠垫い蹇撴缁躲倝鏌ｉ敐鍛伇闁活厽鎹囬弻鈩冨緞鐎ｎ亞浠寸紓浣插亾?
                    ceres::Manifold *manifold = new PoseManifold();
                    problem.AddParameterBlock(statedatalist[k].pose, Adapter::NumPoseParameter(), manifold);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Adapter::NumMixParameter(preintegration_options));
                }

                // GNSS 闂傚倸鍊搁崐鐑芥倿閿曞倸绠栭柛顐ｆ礀绾惧潡鏌ц箛锝呬簼闁告瑥绻橀弻鐔虹磼閵忕姵鐏嶉梺?
                // GNSS factors
                int index = 0;

                ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
                std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id;
                for (const auto &gnss : gnsslist) {
                    auto factor = new GnssFactor(gnss, antlever);
                    for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                        if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                            auto id = problem.AddResidualBlock(factor, loss_function, statedatalist[i].pose);
                            gnss_residualblock_id.push_back(std::make_pair(gnss.time, id));
                            index++;
                            break;
                        }
                    }
                }

                // 婵犵數濮烽。钘壩ｉ崨鏉戠；闁告侗鍙庨悢鍡樹繆椤栨氨姣為柛瀣尭椤繈鎮℃惔銏粚闂備焦鍓氭禍婊堟偉閻撳寒娼栨繛宸簻瀹告繂鈹戦悩鎻掆偓濠氭偡閵娿儮鏀介柍钘夋娴滄繃銇勯妸銉﹀櫧闁告瑥鎳庨埞鎴︽偐鐠囇冧紣闂佺懓鍟块ˇ闈涚暦?                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = Adapter::MakePreintFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                {
                    // IMU 闂傚倸鍊峰ù鍥х暦閸偅鍙忛柡澶嬪殮瑜版帒纾奸柣鎰絻閻庮參姊洪崫鍕垫Ъ妞ゆ洘濞婇幃銏ゅ传閵壯呮闂備礁鎲″ú锕傚储娴犲鏅煫鍥ㄧ⊕閳锋垿鏌熺粙鍨劉妞ゃ儱妫楅埞鎴︻敊閸濆嫧鍋撳Δ浣侯洸?
                    // add IMU bias-constraint factors
                    auto factor = Adapter::MakeImuErrorFactor(preintegrationlist.back());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // 闂傚倸鍊搁崐鐑芥嚄閸洍鈧箓宕奸姀鈥冲簥闂佽澹嗘晶妤呭磻鐎ｎ喗鐓欓柛鎾楀懎绗￠梺绋款儐钃辩紒缁樼洴瀹曞崬螣鐠囪尙顣查梻浣虹帛閸旀洟鏁冮鍫濊摕闁挎繂鎲橀悢鐓庡瀭妞ゆ梻鍋撻妤呮⒒?
                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                // 濠电姷鏁告慨鐢割敊閺嶎厼绐楁慨妯挎硾缁犵娀鏌熼幑鎰靛殭缁炬儳缍婇弻娑欑節閸曨偅鐝￠梺鍝勬閸楀啿顫忕紒妯诲闁告稑锕ラ崕鎾绘⒑缂佹﹩娈旈柛鐔告綑閻ｇ兘骞嬮敃鈧粻鑽ょ磽娴ｅ顏堝焵椤掆偓閻栧ジ鎮￠锕€鐐婇柕濠忕畱閺嗘绱撴担鍝勵€撶紒鑼舵硶濡叉劙骞掑Δ鈧儫闂侀潧绻嗛崜婵嬫偟閺囩姷纾?                // solve the Least-Squares problem
                options.max_num_iterations = num_iterations / 4;
                solver.Solve(options, &problem, &summary);

                // TODO: Just a example, you need remodify.
                // Do GNSS outlier culling using chi-square test
                if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                    // 3 degrees of freedom, 0.05
                    double chi2_threshold = 7.815;

                    // Find GNSS outliers in the window
                    std::unordered_set<double> gnss_outlier;
                    for (size_t k = 0; k < gnsslist.size(); k++) {
                        auto time = gnss_residualblock_id[k].first;
                        auto id   = gnss_residualblock_id[k].second;

                        double cost;
                        double chi2;

                        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
                        chi2 = cost * 2;

                        if (chi2 > chi2_threshold) {
                            gnss_outlier.insert(time);

                            // Reweigthed GNSS
                            double scale = sqrt(chi2 / chi2_threshold);
                            gnsslist[k].std *= scale;
                        }
                    }
                    // // Log outliers
                    // if (!gnss_outlier.empty()) {
                    //     std::string log = absl::StrFormat("Reweight GNSS outlier at %g:", sow - 1);
                    //     for (const auto& time:gnss_outlier) {
                    //         absl::StrAppendFormat(&log, " %g", time);
                    //     }
                    //     std::cout << log << std::endl;
                    // }

                    // Remove all old GNSS factors
                    for (const auto &block : gnss_residualblock_id) {
                        problem.RemoveResidualBlock(block.second);
                    }

                    // Add GNSS factors without loss function
                    index = 0;
                    for (auto &gnss : gnsslist) {
                        auto factor = new GnssFactor(gnss, antlever);
                        for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                            if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                                problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);
                                index++;
                                break;
                            }
                        }
                    }
                }

                options.max_num_iterations = num_iterations * 3 / 4;
                solver.Solve(options, &problem, &summary);

                // 闂傚倸鍊风粈渚€骞栭位鍥敍閻愭潙浜辨繝鐢靛Т濞层倗绮绘导瀛樼厵闂傚倸顕ˇ锕傛煕濮樻剚娼愰柕鍥у楠炴鎹勯悜妯尖偓鐐箾鐎涙ê娈犻柛濠冪墱閹广垹鈽夐姀鐘殿吅闂佽绻戝鎸庣椤忓牄鈧?
                // output the percentage
                int percent            = ((int) sow - starttime) * 100 / (endtime - starttime);
                static int lastpercent = 0;
                if (abs(percent - lastpercent) >= 1) {
                    lastpercent = percent;
                    std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                    flush(std::cout);
                }
            }

            if (preintegrationlist.size() == static_cast<size_t>(windows)) {
                {
                    // 闂傚倸鍊风粈渚€骞栭位鍥敍閻愭潙浜遍梺绯曞墲椤ㄦ劙鎳撻崸妤€绾ч柛顐ｇ濞呭棙銇勯锝嗙闁靛洤瀚伴獮姗€宕￠悙鍏告偅缂傚倷璁查崑?                    // marginalization
                    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                    if (last_marginalization_info && last_marginalization_info->isValid()) {

                        std::vector<int> marginilized_index;
                        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                            if (last_marginalization_parameter_blocks[k] == statedatalist[0].pose ||
                                last_marginalization_parameter_blocks[k] == statedatalist[0].mix) {
                                marginilized_index.push_back(static_cast<int>(k));
                            }
                        }

                        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // 婵犵數濮烽。钘壩ｉ崨鏉戠；闁告侗鍙庨悢鍡樹繆椤栨氨姣為柛瀣尭椤繈鎮℃惔銏粚闂備焦鍓氭禍婊堟偉閻撳寒娼栨繛宸簻瀹告繂鈹戦悩鎻掆偓濠氭偡閵娿儮鏀介柍钘夋娴滄繃銇勯妸銉﹀櫧闁告瑥鎳庨埞鎴︽偐鐠囇冧紣闂佺懓鍟块ˇ闈涚暦?                    // preintegration factors
                    {
                        auto factor   = std::shared_ptr<ceres::CostFunction>(Adapter::MakePreintFactor(preintegrationlist[0]));
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS 闂傚倸鍊搁崐鐑芥倿閿曞倸绠栭柛顐ｆ礀绾惧潡鏌ц箛锝呬簼闁告瑥绻橀弻鐔虹磼閵忕姵鐏嶉梺?
                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    // 闂傚倸鍊风粈渚€骞栭位鍥敃閿曗偓閻ょ偓绻濇繝鍌涘櫧闁活厽鐟╅弻鈥愁吋鎼粹€崇闂侀€炲苯鍘哥紒鑸靛哺閻涱喚鈧綆鍠楅崐鐑芥倵闂堟稒鎲搁懖鏍⒒閸屾瑧绐旀繛鑹板吹閳ь剟娼ч惌鍌氱暦閵徛板亝闁告劑鍔庨悰銉モ攽椤斿浠滈柛瀣崌閺?                    // 闂傚倸鍊风粈渚€骞栭位鍥敃閿曗偓閻ょ偓绻濇繝鍌涘櫧闁活厽鐟╅弻鈥愁吋鎼粹€崇闂侀€炲苯鍘哥紒鑸靛哺閻涱喚鈧綆鍠楅崐鐑芥倵闂堟稒鎲搁懖鏍⒒閸屾瑧绐旀繛鑹板吹閳ь剟娼ч惌鍌氱暦閵徛板亝闁告劑鍔庨悰銉モ攽椤斿浠滈柛瀣崌閺?                    // do marginalization
                    marginalization_info->marginalization();

                    // 闂傚倸鍊搁崐宄懊归崶褜娴栭柕濞炬櫆閸ゅ嫰鏌ょ粙璺ㄤ粵婵炲懐濮垫穱濠囧Χ閸屾矮澹曢梻浣风串缁蹭粙鎮樺杈╃當闁绘梻鍘ч悞鍨亜閹烘垵顏€规挷绶氶弻鐔兼偋閸喓鍑￠梺缁樺姇閿曘儵濡甸崟顖氱鐎广儱娴傚Σ顕€姊虹紒妯肩疄濞存粏娉涢～蹇曠磼濡顎撻梺鑽ゅ枑婢瑰棛绮婃搴ｇ＝濞达絽澹婂Ο鈧┑鐐板尃閸愩劎銈梺鑽ゅ枑缁瞼鎹㈤幋鐘电焿?                    // 闂傚倸鍊搁崐宄懊归崶褜娴栭柕濞炬櫆閸ゅ嫰鏌ょ粙璺ㄤ粵婵炲懐濮垫穱濠囧Χ閸屾矮澹曢梻浣风串缁蹭粙鎮樺杈╃當闁绘梻鍘ч悞鍨亜閹烘垵顏€规挷绶氶弻鐔兼偋閸喓鍑￠梺缁樺姇閿曘儵濡甸崟顖氱鐎广儱娴傚Σ顕€姊虹紒妯肩疄濞存粏娉涢～蹇曠磼濡顎撻梺鑽ゅ枑婢瑰棛绮婃搴ｇ＝濞达絽澹婂Ο鈧┑鐐板尃閸愩劎銈梺鑽ゅ枑缁瞼鎹㈤幋鐘电焿?                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                }

                // 濠电姷鏁告慨鐑藉极閸涘﹥鍙忛柟缁㈠枟閺呮繈鏌曟径鍡樻珔闁搞劌鍊婚幉鎼佹偋閸繄鐟查梺绋款儐閿曘垽寮婚弴鐔风窞婵炴垶姘ㄩ弳鐘电磽娴ｇ顣抽柛瀣ㄥ€濆璇测槈閵忕姴宓嗛梺闈涱焾閸庤京绮诲ú顏呪拺?
                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time)) {
                        gnsslist.pop_front();
                    }
                    timelist.pop_front();
                    preintegrationlist.pop_front();

                    for (int k = 0; k < windows; k++) {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k]     = Adapter::StateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Adapter::StateFromData(statedatalist[windows], preintegration_options);
                    state_curr         = statelist[windows];
                }
            } else {
                state_curr =
                    Adapter::StateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            // write result
            writeNavResult(*timelist.rbegin(), station_origin, state_curr, navfile, errfile);

            // 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢敂缁樻櫈闂佸憡渚楅崹顏堝磻閹炬剚娼╅柣鎾抽椤偆绱撴担浠嬪摵闁圭懓娲悰顔嘉熼崗鐓庣彴闂佸憡鐟ラˇ钘壩涢悢鍏尖拻濞达絿鐡斿鎰亜閺冣偓閻楃姴鐣烽弶璇炬棃宕ㄩ闂存偅闂備礁鎲￠〃鍫ュ磹閿濆應妲堥柕蹇曞Х妤犲洭姊洪崜鑼帥闁革綆鍠楃€靛ジ鍩€椤掑嫭鐓熼柣鎰嚟濞堜即鏌涢妷锝呭闁?            // build a new preintegration object
            preintegrationlist.emplace_back(
                Adapter::UnifiedPreintegrator::Create(parameters, imu_pre, state_curr, preintegration_options,
                                                      imu_pre.is_wheel));
        } else {
            auto integration = *preintegrationlist.rbegin();
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    gnssfile.close();

    auto te = absl::Now();
    std::cout << std::endl << std::endl << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

    return 0;
}

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile) {
    vector<double> result;

    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

void writeNavResultWheel(double time, const Vector3d &origin, const WheelIntegrationState &state, FileSaver &navfile,
                         FileSaver &errfile) {
    vector<double> result;

    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);
    imu00.is_wheel = buff.is_wheel;

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
    imu11.is_wheel = buff.is_wheel;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 闂傚倸鍊搁崐宄懊归崶顒婄稏濠㈣泛顑囬々鎻捗归悩宸剰缁炬儳娼￠幃妤呮濞戞瑥鏆堥悗瑙勬礃閻擄繝鐛弽顬ュ酣顢楅埀顒佷繆閼测晝纾奸柍褜鍓熷畷濂稿煑閳轰椒澹曞Δ鐘靛仜閻忔繈宕濆鑸靛€垫慨妯煎帶濞呭秵銇勯姀鈩冾棃闁哄苯妫楅濂稿幢濞嗗繐绠炲┑鐘殿暯濡插懘宕归幎钘夌？闁靛牆锛嗘径濠庣叆闁割偆鍠撻崢?
        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // 闂傚倸鍊搁崐宄懊归崶顒婄稏濠㈣泛顑囬々鎻捗归悩宸剰缁炬儳娼￠幃妤呮濞戞瑥鏆堥悗瑙勬礃閻擄繝鐛弽顬ュ酣顢楅埀顒佷繆婵傚憡鍋℃繛鍡樼懅閻ｅ灚顨ラ悙瀵稿⒈闁逞屽墯缁嬫帡鈥﹂崶鈺冧笉婵☆垱鐪规禍婊堢叓閸ャ劎鈼ラ柟鏌ョ畺閺岋紕浠﹂崜褉妲堥柧浼欑秮閺岋綁骞嬮悜鍥︾返濠碘槅鍋嗛崑銈咁潖?
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        // 闂傚倸鍊搁崐鎼佸磹閹间礁纾圭紒瀣紩濞差亝鍋愰悹鍥皺閿涙盯姊洪悷鏉库挃缂侇噮鍨跺畷鎴︽晸閻樺磭鍘搁梺鎼炲劘閸斿秹鎯冮幋锔界厱闁哄倽娉曡倴闂佺懓寮堕幃鍌氼嚕閸洖鍨傛い鏃囧Г椤旀帡鏌?        // need interpolation
        return 2;
    }

    return 0;
}



