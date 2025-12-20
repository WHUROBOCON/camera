#include "block_recognizer.hpp"

// 核心识别逻辑：处理所有blocks
FinalBlockResults BlockRecognizer::recognize(const yolo::BoxArray& dets) const {
    FinalBlockResults results;

    // 1. 收集所有 blocks
    std::vector<const yolo::Box*> blocks;
    std::vector<const yolo::Box*> faces;

    for (const auto& d : dets) {
        if (d.class_label == 0) { // block
            blocks.push_back(&d);
        } else {
            faces.push_back(&d);
        }
    }

    // 没有 block 直接返回空
    if (blocks.empty())
        return results;

    // 2. 对每个block进行融合
    for (const auto& block : blocks) {
        cv::Rect block_rect(
            block->left, block->top,
            block->right - block->left,
            block->bottom - block->top
        );

        // 找属于该 block 的 face
        std::set<int> face_classes;
        float face_conf_sum = 0.0f;
        int face_cnt = 0;

        for (auto f : faces) {
            cv::Rect fr(
                f->left, f->top,
                f->right - f->left,
                f->bottom - f->top
            );

            float inter = (block_rect & fr).area();
            if (inter > 0.7f * fr.area()) {
                face_classes.insert(f->class_label);
                face_conf_sum += f->confidence;
                face_cnt++;
            }
        }

            // 判定 block 类别
        BlockClass block_class = BlockClass::UNKNOWN;

        // 如果有检测到的face，根据face类别判定block类别
        if (face_cnt > 0) {
            // R1：所有face都是R1_face (class_label == 1)
            bool all_r1 = true;
            for (int c : face_classes) {
                if (c != 1) {
                    all_r1 = false;
                    break;
                }
            }
            
            if (all_r1) {
                block_class = BlockClass::R1;
            } else {
                // R2r：有2-16范围的face
                bool has_r2r = false;
                for (int c : face_classes) {
                    if (c >= 2 && c <= 16) {
                        has_r2r = true;
                        break;
                    }
                }
                
                // R2f：有17-31范围的face
                bool has_r2f = false;
                for (int c : face_classes) {
                    if (c >= 17 && c <= 31) {
                        has_r2f = true;
                        break;
                    }
                }
                
                // 优先判定R2r，其次R2f
                if (has_r2r) {
                    block_class = BlockClass::R2r;
                } else if (has_r2f) {
                    block_class = BlockClass::R2f;
                }
            }
        }

        // 只有判定出了类别才加入结果
        if (block_class != BlockClass::UNKNOWN) {
            FinalBlockResult result;
            result.detection = *block;
            result.detection.class_label = static_cast<int>(block_class);
            
            // 只返回有face关联的block
            if (face_cnt > 0) {
                // 融合block和face的置信度
                result.confidence = 0.5f * block->confidence + 0.5f * (face_conf_sum / face_cnt);
                result.detection.confidence = result.confidence;
                result.block_class = block_class;
                results.push_back(result);
            }
            // 否则不返回这个孤立的block
        }
    }

    return results;
}


