#include "dict.h"

#define DICT_BOOL   1
#define DICT_INT    2
#define DICT_DOUBLE 3
#define DICT_STRING 4
#define DICT_VEC    5
#define DICT_MAT    6
#define DICT_CUBE   7
#define DICT_DICT   8

bool dict::hasKey(const std::string &key) {
  std::map<std::string, int>::iterator iter;
  iter = this->keys.find(key);
  return iter != this->keys.end();
}

void dict::setBool(const std::string &key, const bool &value) {
  this->keys[key] = this->bools.size();
  this->type[key] = DICT_BOOL;
  this->bools.push_back(value);
}

void dict::setInt(const std::string &key, const int &value) {
  this->keys[key] = this->ints.size();
  this->type[key] = DICT_INT;
  this->ints.push_back(value);
}

void dict::setDouble(const std::string &key, const double &value) {
  this->keys[key] = this->doubles.size();
  this->type[key] = DICT_DOUBLE;
  this->doubles.push_back(value);
}

void dict::setString(const std::string &key, const std::string &value) {
  this->keys[key] = this->strings.size();
  this->type[key] = DICT_STRING;
  this->strings.push_back(value);
}

void dict::setVec(const std::string &key, const arma::vec &value) {
  this->keys[key] = this->fvecs.size();
  this->type[key] = DICT_VEC;
  this->fvecs.push_back(value);
}

void dict::setMat(const std::string &key, const arma::mat &value) {
  this->keys[key] = this->fmats.size();
  this->type[key] = DICT_MAT;
  this->fmats.push_back(value);
}

void dict::setCube(const std::string &key, const arma::cube &value) {
  this->keys[key] = this->fcubes.size();
  this->type[key] = DICT_CUBE;
  this->fcubes.push_back(value);
}

void dict::setDict(const std::string &key, const dict &value) {
  this->keys[key] = this->dicts.size();
  this->type[key] = DICT_DICT;
  this->dicts.push_back(value);
}

bool dict::getBool(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_BOOL) {
    return false;
  }
  return this->bools[this->keys[key]];
}

int dict::getInt(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_INT) {
    return 0;
  }
  return this->ints[this->keys[key]];
}

double dict::getDouble(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_DOUBLE) {
    return 0.0;
  }
  return this->doubles[this->keys[key]];
}

std::string dict::getString(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_STRING) {
    return std::string();
  }
  return this->strings[this->keys[key]];
}

arma::vec dict::getVec(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_VEC) {
    return arma::vec();
  }
  return this->fvecs[this->keys[key]];
}

arma::mat dict::getMat(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_MAT) {
    return arma::mat();
  }
  return this->fmats[this->keys[key]];
}

arma::cube dict::getCube(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_CUBE) {
    return arma::cube();
  }
  return this->fcubes[this->keys[key]];
}

dict dict::getDict(const std::string &key) {
  if (!this->hasKey(key) || this->type[key] != DICT_DICT) {
    return dict();
  }
  return this->dicts[this->keys[key]];
}
