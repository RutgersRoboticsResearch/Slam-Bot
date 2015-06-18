#ifndef __SB_DICT_H__
#define __SB_DICT_H__

#include <vector>
#include <map>
#include <armadillo>

class dict {
  private:
    std::vector<bool> bools;
    std::vector<int> ints;
    std::vector<double> doubles;
    std::vector<std::string> strings;
    std::vector<arma::vec> fvecs;
    std::vector<arma::mat> fmats;
    std::vector<arma::cube> fcubes;
    std::vector<dict> dicts;
    std::map<std::string, int> keys;
    std::map<std::string, int> type;

  public:
    bool hasKey(const std::string &key);

    void setBool(const std::string &key, const bool &value);
    void setInt(const std::string &key, const int &value);
    void setDouble(const std::string &key, const double &value);
    void setString(const std::string &key, const std::string &value);
    void setVec(const std::string &key, const arma::vec &value);
    void setMat(const std::string &key, const arma::mat &value);
    void setCube(const std::string &key, const arma::cube &value);
    void setDict(const std::string &key, const dict &value);

    bool getBool(const std::string &key);
    int getInt(const std::string &key);
    double getDouble(const std::string &key);
    std::string getString(const std::string &key);
    arma::vec getVec(const std::string &key);
    arma::mat getMat(const std::string &key);
    arma::cube getCube(const std::string &key);
    dict getDict(const std::string &key);
};

#endif
