#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#define NAXIS_HTML_ROOT "."
#include "naxis_html_dump.h"

int main() {
  const char* output_path = "tests/out/square.html";
  std::filesystem::create_directories("tests/out");
  std::filesystem::remove(output_path);
  NaxisHtmlDump dump("square", "F5 square X < Y & Z > 0");
  dump.row(0.125, 3.0, -2.0);
  dump.finish();

  std::ifstream file(output_path);
  const std::string page{std::istreambuf_iterator<char>(file), {}};
  const char* expected[] = {
      "<title>F5 square X &lt; Y &amp; Z &gt; 0</title>",
      "<h1>F5 square X &lt; Y &amp; Z &gt; 0</h1>",
      "<pre id=\"trace\">0.125000 3.000000 -2.000000\n</pre>",
  };
  for (const auto* text : expected) {
    if (page.find(text) == std::string::npos) {
      std::cerr << "Missing trace output: " << text << '\n';
      return 1;
    }
  }
  return 0;
}
