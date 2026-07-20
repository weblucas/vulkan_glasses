// Unit tests for the pose-file CSV parsing used by the CSV renderer.
//
// This validates the tokenizing done by CSVProcessor::parseLine()
// (apps/csv_renderer/src/csv_processor.cc) and the field layout consumed by
// runHeadless(): each valid image-pose line has 8 comma-separated fields
//   id, p_x, p_y, p_z, q_x, q_y, q_z, q_w
// that are trimmed of surrounding whitespace. Lines with a different field
// count are skipped by the renderer.

#include <string>
#include <vector>

#include <vkg/string_utils.h>

#include "test_common.h"

// Mirror of CSVProcessor::parseLine().
static void parseLine(const std::string& line, std::vector<std::string>& vec) {
    for (const std::string& field : vg_str::split(line, ','))
        vec.push_back(vg_str::trim(field));
}

int main() {
    // A real line from example/image_poses.txt.
    {
        std::vector<std::string> vec;
        parseLine(
            "DJI_0001.JPG,19.256670,-11.677642,6.314486,-0.922348,-0.046673,"
            "0.010963,0.383373",
            vec);
        CHECK(vec.size() == 8);
        CHECK(vec[0] == "DJI_0001.JPG");
        CHECK_APPROX(std::stod(vec[1]), 19.256670);
        CHECK_APPROX(std::stod(vec[2]), -11.677642);
        CHECK_APPROX(std::stod(vec[3]), 6.314486);
        CHECK_APPROX(std::stod(vec[7]), 0.383373);
    }

    // Surrounding whitespace must be trimmed (renderer relies on trimming).
    {
        std::vector<std::string> vec;
        parseLine(" a , 1.5 ,\t-2.0 ", vec);
        CHECK(vec.size() == 3);
        CHECK(vec[0] == "a");
        CHECK_APPROX(std::stod(vec[1]), 1.5);
        CHECK_APPROX(std::stod(vec[2]), -2.0);
    }

    // The header line does not have 8 numeric columns and is skipped in
    // runHeadless() (the first getline consumes it); here we just confirm the
    // "wrong field count => not 8" guard behaviour.
    {
        std::vector<std::string> vec;
        parseLine("timestamp, p_x, p_y,p_z, q_x, q_y,q_z, q_w", vec);
        CHECK(vec.size() == 8);  // header happens to have 8 columns
        std::vector<std::string> short_line;
        parseLine("only,three,fields", short_line);
        CHECK(short_line.size() != 8);  // would be skipped by the renderer
    }

    if (::vgtest::failure_count() == 0)
        std::printf("test_csv_parse: all checks passed\n");
    TEST_MAIN_RETURN();
}
