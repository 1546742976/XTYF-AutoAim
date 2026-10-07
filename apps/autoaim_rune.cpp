#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
  constexpr const char* unavailable =
      "Rune mission is not implemented. No pipeline or device was started.";
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0] << " --help\n" << unavailable << '\n';
    return 0;
  }

  std::cerr << unavailable << '\n';
  return 2;
}
