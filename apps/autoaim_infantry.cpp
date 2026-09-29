#include "autoaim/pipeline/entry.hpp"

int main(int argc, char** argv) {
  return autoaim::pipeline::run_entry(argc, argv, autoaim::core::Role::infantry);
}
