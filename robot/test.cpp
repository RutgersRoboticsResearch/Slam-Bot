#include <njson/json.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;
using json = nlohmann::json;

int main() {
  FILE *jsonfile;
  jsonfile = fopen("out.json", "w");
  json j = json::object();
  j["id"]["name"] = "world";
  fprintf(jsonfile, "%s", j.dump().c_str());
  return 0;
}
