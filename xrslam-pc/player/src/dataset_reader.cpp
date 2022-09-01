#include <async_dataset_reader.h>
#include <dataset_reader.h>
#include <euroc_dataset_reader.h>
#include <optional>
#include <tum_dataset_reader.h>

std::optional<std::string> path_from_scheme(const std::string &string,
                                            const std::string &pattern) {
    if (string.length() >= pattern.length()) {
        if (string.substr(0, pattern.length()) == pattern) {
            return string.substr(pattern.length());
        }
    }
    return {};
}

std::unique_ptr<DatasetReader>
DatasetReader::create_reader(const std::string &filename, bool async) {
    std::unique_ptr<DatasetReader> reader;
    if (auto path = path_from_scheme(filename, "euroc://")) {
        reader = std::make_unique<EurocDatasetReader>(path.value());
    } else if (auto path = path_from_scheme(filename, "tum://")) {
        reader = std::make_unique<TUMDatasetReader>(path.value());
    } else {
        return nullptr;
    }
    if (async) {
        return std::make_unique<AsyncDatasetReader>(std::move(reader));
    } else {
        return reader;
    }
}
