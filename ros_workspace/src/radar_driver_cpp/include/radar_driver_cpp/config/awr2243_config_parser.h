#ifndef AWR2243_PARSER_H
#define AWR2243_PARSER_H

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <map>
#include <stdexcept>

struct ADC_PARAMS {
    double startFreq;
    double idleTime;
    double adc_valid_start_time;
    double rampEndTime;
    double freq_slope;
    double txStartTime;
    int samples;
    int sample_rate;
    int chirps;
    double frame_periodicity;
    int rx;
    int tx;
    int IQ;
    int bytes;
    int frameCount;
    std::string triggerSelect;
};

struct CFG_PARAMS {
    int txAntMask;
    int numTxChan;
    int rxAntMask;
    int numRxChan;
    int lvdsBW;
    int numlaneEn;
};

inline void trim(std::string &s) {
    s.erase(0, s.find_first_not_of(" \t\n\r"));
    s.erase(s.find_last_not_of(" \t\n\r") + 1);
}

inline std::vector<std::string> split(const std::string &s, const char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline ADC_PARAMS awr2243_read_config(const std::string &config_file_name) {
    ADC_PARAMS adc_params{};
    CFG_PARAMS cfg_params{};

    std::ifstream config(config_file_name);
    if (!config.is_open()) {
        throw std::runtime_error("Unable to open config file");
    }

    std::string line;
    int cur_profile_id = 0;
    int chirp_start_idx_fcf = 0, chirp_end_idx_fcf = 0, loop_count = 0;
    std::vector<int> chirp_start_idx_buf, chirp_end_idx_buf, profile_id_cpcfg_buf, tx_enable_buf;

    while (getline(config, line)) {
        trim(line);
        if (line.empty() || line[0] == '#') continue;

        auto list_match = split(line, '=');
        if (list_match.size() != 2) continue; // skip invalid lines

        std::string key = list_match[0];
        std::string value = split(list_match[1], ';')[0];

        if (key == "channelTx") {
            cfg_params.txAntMask = std::stoi(value);
            cfg_params.numTxChan = 0;
            for (int i = 0; i < 3; ++i) {
                if ((cfg_params.txAntMask >> i) & 1) cfg_params.numTxChan++;
            }
        } else if (key == "rxChanEn" || key == "channelRx") {
            cfg_params.rxAntMask = std::stoi(value);
            cfg_params.numRxChan = 0;
            for (int i = 0; i < 4; ++i) {
                if ((cfg_params.rxAntMask >> i) & 1) cfg_params.numRxChan++;
            }
            adc_params.rx = cfg_params.numRxChan;
        } else if (key == "adcBitsD") {
            adc_params.bytes = std::stoi(value);
        } else if (key == "adcFmt") {
            adc_params.IQ = (std::stoi(value) == 1 || std::stoi(value) == 2) ? 2 : 1;
        } else if (key == "dataRate") {
            int data_rate = std::stoi(value);
            std::map<int, int> data_rate_map = {
                {0b0001, 600}, {0b0010, 450}, {0b0011, 400},
                {0b0100, 300}, {0b0101, 225}, {0b0110, 150}
            };
            cfg_params.lvdsBW = data_rate_map[data_rate];
        } else if (key == "laneEn") {
            int lane_en = std::stoi(value);
            cfg_params.numlaneEn = 0;
            for (int i = 0; i < 4; ++i) {
                if ((lane_en >> i) & 1) cfg_params.numlaneEn++;
            }
        } else if (key == "profileId") {
            cur_profile_id = std::stoi(value);
        } else if (key == "startFreqConst" && cur_profile_id == 0) {
            adc_params.startFreq = std::stoi(value) * (3.6 / (1 << 26));
        } else if (key == "idleTimeConst" && cur_profile_id == 0) {
            adc_params.idleTime = std::stoi(value) / 100.0;
        } else if (key == "adcStartTimeConst" && cur_profile_id == 0) {
            adc_params.adc_valid_start_time = std::stoi(value) / 100.0;
        } else if (key == "rampEndTime" && cur_profile_id == 0) {
            adc_params.rampEndTime = std::stoi(value) / 100.0;
        } else if (key == "freqSlopeConst" && cur_profile_id == 0) {
            adc_params.freq_slope = std::stoi(value) * (3.6e3 * 900 / (1 << 26));
        } else if (key == "txStartTime" && cur_profile_id == 0) {
            adc_params.txStartTime = std::stoi(value) / 100.0;
        } else if (key == "numAdcSamples" && cur_profile_id == 0) {
            adc_params.samples = std::stoi(value);
        } else if (key == "digOutSampleRate" && cur_profile_id == 0) {
            adc_params.sample_rate = std::stoi(value);
        } else if (key == "rxGain") {
            cur_profile_id++;
        } else if (key == "chirpStartIdx") {
            chirp_start_idx_buf.push_back(std::stoi(value));
        } else if (key == "chirpEndIdx") {
            chirp_end_idx_buf.push_back(std::stoi(value));
        } else if (key == "profileIdCPCFG") {
            profile_id_cpcfg_buf.push_back(std::stoi(value));
        } else if (key == "txEnable") {
            tx_enable_buf.push_back(std::stoi(value));
        } else if (key == "chirpStartIdxFCF") {
            chirp_start_idx_fcf = std::stoi(value);
        } else if (key == "chirpEndIdxFCF") {
            chirp_end_idx_fcf = std::stoi(value);
        } else if (key == "loopCount") {
            loop_count = std::stoi(value);
        } else if (key == "periodicity") {
            adc_params.frame_periodicity = std::stoi(value) / 200000.0;
        } else if (key == "frameCount") {
            adc_params.frameCount = std::stoi(value);
        } else if (key == "numAdcSamples") {
            if (adc_params.IQ == 2 && std::stoi(value) / 2 != adc_params.samples) {
                throw std::runtime_error("numAdcSamples in rlframe_t should be twice the value in rlprofile_t for Complex.");
            }
        } else if (key == "triggerSelect") {
            adc_params.triggerSelect = (std::stoi(value) == 1) ? "software" : "hardware";
        }
    }

    adc_params.tx = tx_enable_buf.size();
    if (adc_params.tx > cfg_params.numTxChan) {
        throw std::runtime_error("Exceeded max tx num, check channelTx and chirp cfg.");
    }

    int tmp_chirp_num = chirp_end_idx_buf[0] - chirp_start_idx_buf[0] + 1;

    if (int all_chirp_num = tmp_chirp_num * adc_params.tx; all_chirp_num != chirp_end_idx_fcf - chirp_start_idx_fcf + 1) {
        throw std::runtime_error("Chirp number mismatch between frame cfg and chirp cfg.");
    }

    adc_params.chirps = loop_count * tmp_chirp_num;
    return adc_params;
}

inline void print_params(const ADC_PARAMS &adc_params) {
    std::cout << "ADC_PARAMS:" << std::endl;
    std::cout << "  startFreq: " << adc_params.startFreq << std::endl;
    std::cout << "  idleTime: " << adc_params.idleTime << std::endl;
    std::cout << "  adc_valid_start_time: " << adc_params.adc_valid_start_time << std::endl;
    std::cout << "  rampEndTime: " << adc_params.rampEndTime << std::endl;
    std::cout << "  freq_slope: " << adc_params.freq_slope << std::endl;
    std::cout << "  txStartTime: " << adc_params.txStartTime << std::endl;
    std::cout << "  samples: " << adc_params.samples << std::endl;
    std::cout << "  sample_rate: " << adc_params.sample_rate << std::endl;
    std::cout << "  chirps: " << adc_params.chirps << std::endl;
    std::cout << "  frame_periodicity: " << adc_params.frame_periodicity << std::endl;
    std::cout << "  rx: " << adc_params.rx << std::endl;
    std::cout << "  tx: " << adc_params.tx << std::endl;
    std::cout << "  IQ: " << adc_params.IQ << std::endl;
    std::cout << "  bytes: " << adc_params.bytes << std::endl;
    std::cout << "  triggerSelect: " << adc_params.triggerSelect << std::endl;
}

#endif // AWR2243_PARSER_H
