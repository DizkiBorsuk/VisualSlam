/**
 * @file data_parser.hpp
 * @author mrostocki
 * @brief
 * @version 0.1
 * @date 2024-07-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "mrVSLAM/common_includes.hpp"

namespace mrVSLAM
{
    class YamlParser
    {
    public:

        YamlParser(const std::string filepath) {
            openFile(filepath, &fs_);
        }
        ~YamlParser() { closeFile(&fs_); }

        template <class T>
        void getYamlParam(const std::string& id, T* output) const {
            if(!id.empty()) {
                fmt::print("sring empty \n");
                return;
            }
            const cv::FileNode& file_handle = fs_[id];

            if(file_handle.type() == cv::FileNode::NONE)
            {
                fmt::print("file handle empty \n");
                return;
            }
            file_handle >> (output);
        }

        template <class T>
        void getNestedYamlParam(const std::string& id,
                                const std::string& id_2,
                                T* output) const {

            const cv::FileNode& file_handle = fs_[id];

            if(file_handle.type() == cv::FileNode::NONE) {
                fmt::print("file handle empty \n");
                return;
            }

            const cv::FileNode& file_handle_2 = file_handle[id_2];

            if(file_handle_2.type() == cv::FileNode::NONE) {
                fmt::print("file handle 2 empty \n");
                return;
            }

            // CHECK(file_handle.isMap())
            //     << "I think that if this is not a map, we can't use >>";
            file_handle_2 >> (output);
        }

        inline bool hasParam(const std::string& id) const {
            const auto& handle = fs_[id];
            return handle.type() != cv::FileNode::NONE;
        }

    private:
        void openFile(const std::string& filepath, cv::FileStorage* fs) const {
            if(!filepath.empty()){
                fmt::print("Empty filepath!");
                return;
            }
            try {
                (fs)->open(filepath, cv::FileStorage::READ);
            } catch (cv::Exception& e) {
                fmt::print("Cannot open file: {}, OpenCV error code: {}", filepath, e.msg);
            }

        }

        inline void closeFile(cv::FileStorage* fs) const {
            (fs)->release();
        }

    private:
        cv::FileStorage fs_;
        std::string filepath_;

    };
}
