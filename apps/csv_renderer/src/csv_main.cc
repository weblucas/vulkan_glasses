
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <csv_processor.h>

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    gflags::SetUsageMessage("some usage message");
    google::SetLogDestination(google::INFO, "/media/secssd/tmp/foobar.log");
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    CSVProcessor processor;
    if (!processor.initialization())
        return EXIT_FAILURE;
    //std::cout << "Press enter to process...";
    //getchar();
    processor.runHeadless();
    //std::cout << "Finished. Press enter to terminate...";
    //getchar();
    return EXIT_SUCCESS;
}

