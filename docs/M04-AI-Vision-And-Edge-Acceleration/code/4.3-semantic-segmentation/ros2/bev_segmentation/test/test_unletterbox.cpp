// test/test_unletterbox.cpp
//
// Geometry tests for unletterbox_mask() — the inverse of the preprocessing
// letterbox.
//
// These exist because the node used to resize the *padded* argmax mask straight
// to the original resolution, which drags letterbox padding onto real pixels.
// Every test below asserts BOTH paths:
//
//   * the restored mask comes out clean (the fix)
//   * the old nearest_neighbor_resize path demonstrably does not (the bug)
//
// If the second half of a test ever stops failing, the test has stopped
// discriminating and is no longer evidence of anything.
//
// No TensorRT / CUDA needed — geometry only.

#include <gtest/gtest.h>

#include "bev_segmentation/postprocess.hpp"
#include "bev_segmentation/preprocess.hpp"
#include "bev_segmentation/types.hpp"

using namespace bev_segmentation;

namespace {

// kModelInput 1024x512 is 2:1. A 16:9 camera (1920x1080 / 1280x720) therefore
// letterboxes horizontally; a 4:3 source pads harder; a wider-than-2:1 source
// pads vertically instead. All three cases are covered below.

constexpr uint8_t kPadClass = 1;      // stands in for whatever the model predicts on padding
constexpr uint8_t kContentClass = 2;  // stands in for a real scene class

// A mask whose leading `pad_cols` columns (and optionally leading `pad_rows`
// rows) are padding, everything else content.
std::vector<uint8_t> make_marked_mask(int pad_cols, int pad_rows = 0) {
    std::vector<uint8_t> m(static_cast<size_t>(kLogitsH) * kLogitsW, kContentClass);
    for (int y = 0; y < kLogitsH; ++y) {
        for (int x = 0; x < kLogitsW; ++x) {
            if (x < pad_cols || y < pad_rows) {
                m[static_cast<size_t>(y) * kLogitsW + x] = kPadClass;
            }
        }
    }
    return m;
}

int count_class(const std::vector<uint8_t>& m, uint8_t cls) {
    int n = 0;
    for (uint8_t v : m) {
        if (v == cls) ++n;
    }
    return n;
}

// Count how many entries of row 0 are `cls`.
int count_class_in_row0(const std::vector<uint8_t>& m, int w, uint8_t cls) {
    int n = 0;
    for (int x = 0; x < w; ++x) {
        if (m[static_cast<size_t>(x)] == cls) ++n;
    }
    return n;
}

}  // namespace

// ---------------------------------------------------------------------------
// The letterbox geometry contract itself. If these numbers move, every other
// test here is measuring the wrong thing.
// ---------------------------------------------------------------------------
TEST(LetterboxGeometryTest, PaddingValuesPerResolution) {
    // 1920x1080 and 1280x720 are both 16:9, so they letterbox identically —
    // asserted separately because they are the two real camera modes.
    LetterboxMeta hd;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, hd);
    EXPECT_NEAR(hd.scale, 0.474074f, 1e-5f);
    EXPECT_EQ(hd.pad_x, 57);
    EXPECT_EQ(hd.pad_y, 0);

    LetterboxMeta hd720;
    compute_letterbox(720, 1280, kModelInputH, kModelInputW, hd720);
    EXPECT_NEAR(hd720.scale, 0.711111f, 1e-5f);
    EXPECT_EQ(hd720.pad_x, 57);
    EXPECT_EQ(hd720.pad_y, 0);

    LetterboxMeta vga;
    compute_letterbox(480, 640, kModelInputH, kModelInputW, vga);
    EXPECT_NEAR(vga.scale, 1.066667f, 1e-5f);
    EXPECT_EQ(vga.pad_x, 170);
    EXPECT_EQ(vga.pad_y, 0);
}

// ---------------------------------------------------------------------------
// The discriminating tests: padding must not reach the output.
// ---------------------------------------------------------------------------
TEST(UnletterboxTest, RemovesHorizontalPadding1920x1080) {
    LetterboxMeta meta;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, meta);
    ASSERT_EQ(meta.pad_x, 57);

    const int pad_cols = meta.pad_x / (kModelInputW / kLogitsW);  // 57/4 = 14
    ASSERT_EQ(pad_cols, 14);
    const auto mask = make_marked_mask(pad_cols);

    const auto fixed = unletterbox_mask(mask, meta, 1080, 1920);
    ASSERT_EQ(fixed.size(), static_cast<size_t>(1080) * 1920u);

    // The canvas column for source column 0 is pad_x itself, which already sits
    // inside the content region — so no output pixel should be padding at all.
    EXPECT_EQ(count_class(fixed, kPadClass), 0)
        << "restored mask still contains letterbox padding";

    // The old path: resize the padded mask 128x256 -> 1080x1920 directly.
    // Mask column 0 (padding) covers output columns 0..(1920*14/256 - 1) = 104.
    const auto buggy = nearest_neighbor_resize(mask, kLogitsH, kLogitsW, 1080, 1920);
    const int buggy_pad_cols = count_class_in_row0(buggy, 1920, kPadClass);
    EXPECT_GT(buggy_pad_cols, 50)
        << "expected the old path to smear ~105 padding columns into row 0; "
           "got " << buggy_pad_cols << " — the test has stopped discriminating";
}

TEST(UnletterboxTest, RemovesHorizontalPadding1280x720) {
    LetterboxMeta meta;
    compute_letterbox(720, 1280, kModelInputH, kModelInputW, meta);
    ASSERT_EQ(meta.pad_x, 57);

    const int pad_cols = meta.pad_x / (kModelInputW / kLogitsW);
    const auto mask = make_marked_mask(pad_cols);

    const auto fixed = unletterbox_mask(mask, meta, 720, 1280);
    EXPECT_EQ(count_class(fixed, kPadClass), 0);

    const auto buggy = nearest_neighbor_resize(mask, kLogitsH, kLogitsW, 720, 1280);
    EXPECT_GT(count_class_in_row0(buggy, 1280, kPadClass), 50);
}

TEST(UnletterboxTest, RemovesHorizontalPadding640x480) {
    LetterboxMeta meta;
    compute_letterbox(480, 640, kModelInputH, kModelInputW, meta);
    ASSERT_EQ(meta.pad_x, 170);

    const int pad_cols = meta.pad_x / (kModelInputW / kLogitsW);  // 170/4 = 42
    ASSERT_EQ(pad_cols, 42);
    const auto mask = make_marked_mask(pad_cols);

    const auto fixed = unletterbox_mask(mask, meta, 480, 640);
    EXPECT_EQ(count_class(fixed, kPadClass), 0);

    // Buggy: mask column 0 covers output columns 0..(640*42/256 - 1) = 104.
    const auto buggy = nearest_neighbor_resize(mask, kLogitsH, kLogitsW, 480, 640);
    EXPECT_GT(count_class_in_row0(buggy, 640, kPadClass), 50);
}

// Sources wider than 2:1 pad vertically instead. The real camera never does
// this, so it is the only test that exercises the y branch of the restore.
TEST(UnletterboxTest, RemovesVerticalPaddingUltraWideSource) {
    const int src_h = 480, src_w = 1920;   // 4:1
    LetterboxMeta meta;
    compute_letterbox(src_h, src_w, kModelInputH, kModelInputW, meta);

    EXPECT_EQ(meta.pad_x, 0);
    ASSERT_EQ(meta.pad_y, 128) << "4:1 source should letterbox vertically";
    // LetterboxMeta carries scale/pad only; derive the scaled height the same
    // way letterbox_to_chw_float() does.
    EXPECT_EQ(static_cast<int>(src_h * meta.scale + 0.5f), 256);

    const int pad_rows = meta.pad_y / (kModelInputH / kLogitsH);  // 128/4 = 32
    ASSERT_EQ(pad_rows, 32);
    const auto mask = make_marked_mask(0, pad_rows);

    const auto fixed = unletterbox_mask(mask, meta, src_h, src_w);
    ASSERT_EQ(fixed.size(), static_cast<size_t>(src_h) * src_w);
    EXPECT_EQ(count_class(fixed, kPadClass), 0)
        << "restored mask still contains vertical letterbox padding";

    // Buggy: mask row 0 covers output rows 0..(480*32/128 - 1) = 119.
    const auto buggy = nearest_neighbor_resize(mask, kLogitsH, kLogitsW, src_h, src_w);
    int buggy_pad_rows = 0;
    for (int y = 0; y < src_h; ++y) {
        if (buggy[static_cast<size_t>(y) * src_w] == kPadClass) ++buggy_pad_rows;
    }
    EXPECT_GT(buggy_pad_rows, 50)
        << "expected the old path to smear ~120 padding rows into column 0";
}

// A 2:1 source needs no padding at all, so restore must be a pure resize and
// must agree with the old path exactly. This is a regression guard: the fix
// must not perturb the one case that already worked.
TEST(UnletterboxTest, NoPaddingDegeneratesToPlainResize) {
    const int src_h = 512, src_w = 1024;   // exactly 2:1
    LetterboxMeta meta;
    compute_letterbox(src_h, src_w, kModelInputH, kModelInputW, meta);
    ASSERT_EQ(meta.pad_x, 0);
    ASSERT_EQ(meta.pad_y, 0);
    EXPECT_NEAR(meta.scale, 1.0f, 1e-6f);

    // A gradient so any index mapping difference shows up.
    std::vector<uint8_t> mask(static_cast<size_t>(kLogitsH) * kLogitsW);
    for (int y = 0; y < kLogitsH; ++y) {
        for (int x = 0; x < kLogitsW; ++x) {
            mask[static_cast<size_t>(y) * kLogitsW + x] =
                static_cast<uint8_t>((x / 16) % kNumClasses);
        }
    }

    const auto restored = unletterbox_mask(mask, meta, src_h, src_w);
    const auto plain = nearest_neighbor_resize(mask, kLogitsH, kLogitsW, src_h, src_w);
    ASSERT_EQ(restored.size(), plain.size());
    for (size_t i = 0; i < restored.size(); ++i) {
        ASSERT_EQ(restored[i], plain[i]) << "index " << i;
    }
}

// Spatial accuracy, not just padding removal: a single distinctive class must
// land within a couple of pixels of where the geometry says it belongs.
TEST(UnletterboxTest, MarkerLandsWhereGeometrySaysItShould) {
    const int src_h = 1080, src_w = 1920;
    LetterboxMeta meta;
    compute_letterbox(src_h, src_w, kModelInputH, kModelInputW, meta);

    // Mark mask row 8 (near the top edge, where the old offset error is large).
    constexpr int kMarkRow = 8;
    std::vector<uint8_t> mask(static_cast<size_t>(kLogitsH) * kLogitsW, kContentClass);
    for (int x = 0; x < kLogitsW; ++x) {
        mask[static_cast<size_t>(kMarkRow) * kLogitsW + x] = 3;
    }

    const auto fixed = unletterbox_mask(mask, meta, src_h, src_w);

    // The marked mask row covers canvas rows [kMarkRow*4, kMarkRow*4+4).
    // canvas_y = pad_y + round(oy * scale), so the expected source row band is
    // round((32 - 0) / scale) .. round((35 - 0) / scale).
    const int expect_lo = static_cast<int>(kMarkRow * (kModelInputH / kLogitsH) / meta.scale);
    const int expect_hi = static_cast<int>((kMarkRow * (kModelInputH / kLogitsH) + 4) / meta.scale);

    int first = -1, last = -1;
    for (int y = 0; y < src_h; ++y) {
        if (fixed[static_cast<size_t>(y) * src_w] == 3) {
            if (first < 0) first = y;
            last = y;
        }
    }
    ASSERT_GE(first, 0) << "marker row vanished from the restored mask";
    EXPECT_GE(first, expect_lo - 2);
    EXPECT_LE(last, expect_hi + 2);
    EXPECT_NEAR((first + last) / 2, (expect_lo + expect_hi) / 2, 2)
        << "marker band centre drifted: got [" << first << "," << last
        << "] expected about [" << expect_lo << "," << expect_hi << "]";
}

// ---------------------------------------------------------------------------
// Guards
// ---------------------------------------------------------------------------
TEST(UnletterboxTest, RejectsMaskOfWrongSize) {
    LetterboxMeta meta;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, meta);
    std::vector<uint8_t> too_small(100, kContentClass);
    EXPECT_THROW(unletterbox_mask(too_small, meta, 1080, 1920), std::invalid_argument);
}

TEST(UnletterboxTest, RejectsMismatchedCanvas) {
    LetterboxMeta meta;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, meta);
    meta.dst_w = 512;   // no longer the size the engine consumes
    const auto mask = make_marked_mask(14);
    EXPECT_THROW(unletterbox_mask(mask, meta, 1080, 1920), std::invalid_argument);
}

TEST(UnletterboxTest, RejectsNonPositiveScale) {
    LetterboxMeta meta;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, meta);
    meta.scale = 0.0f;
    const auto mask = make_marked_mask(14);
    EXPECT_THROW(unletterbox_mask(mask, meta, 1080, 1920), std::invalid_argument);
}

TEST(UnletterboxTest, RejectsBadOutputDims) {
    LetterboxMeta meta;
    compute_letterbox(1080, 1920, kModelInputH, kModelInputW, meta);
    const auto mask = make_marked_mask(14);
    EXPECT_THROW(unletterbox_mask(mask, meta, 0, 1920), std::invalid_argument);
    EXPECT_THROW(unletterbox_mask(mask, meta, 1080, -1), std::invalid_argument);
}