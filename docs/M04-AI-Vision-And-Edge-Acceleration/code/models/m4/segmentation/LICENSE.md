# License Status — M4.3 SegFormer Segmentation

## Upstream Checkpoint
- Name: `nvidia/segformer-b0-finetuned-cityscapes-512-1024`
- Hosted at: HuggingFace Hub (https://huggingface.co/nvidia/segformer-b0-finetuned-cityscapes-512-1024)

## ✅ Status: VERIFIED (2026-09-18)

Verified against the live model metadata, not against a mirror of the
repository's own claims.

## Verified License

```
License:        "other" — no SPDX identifier, no license_name, no license_link
Verified by:    automated metadata query during the M4.3 recovery pass
Verified date:  2026-09-18
HF page link:   https://huggingface.co/nvidia/segformer-b0-finetuned-cityscapes-512-1024
How verified:   GET https://hf-mirror.com/api/models/nvidia/segformer-b0-finetuned-cityscapes-512-1024
                -> tags: ["license:other"], cardData.license: "other",
                   cardData.license_name: null, cardData.license_link: null
                and GET .../raw/main/README.md -> frontmatter `license: other`,
                   with no licence text, link, or terms anywhere in the card.
Notes:          NON-PERMISSIVE AND UNSPECIFIED.
```

### What this actually means

The card declares `other` and then never says what "other" is. The card is
authored by the Hugging Face team, not NVIDIA ("the team releasing SegFormer did
not write a model card for this model") — so the card is not an authoritative
statement of NVIDIA's terms either.

Consequences, and they are not negotiable without a human doing legal review:

- **Do not describe this checkpoint as MIT, Apache-2.0, or commercial-friendly.**
  `other` is not a permissive identifier.
- **Do not redistribute the weights.** `models/m4/segmentation/` therefore ships
  only this file plus `README.md`; the `.onnx` and `.engine` are gitignored and
  must be regenerated locally by each user from the upstream checkpoint.
- Anything derived from the checkpoint (`.onnx`, `.engine`) inherits its terms.
- The upstream code lives at https://github.com/NVlabs/SegFormer; its license is
  a separate question from this checkpoint's and is **NOT VERIFIED here**.

**Release statement required:** "License verified before release" — the
verification above is a metadata check, and it establishes only that the terms
are unspecified. Publication requires a human to resolve them with NVIDIA.

## Internal Assets
- 本仓库 `models/m4/segmentation/` 目录下生成的 `.onnx` 与 `.engine` 为 target-specific
  binary artifact，**不可作为 portable source artifact**，license 仅覆盖上游 checkpoint。
- C++ 代码 (`4.3-semantic-segmentation/ros2/bev_segmentation/`) 声明为 **MIT**
  (`package.xml`)，与同模块的 bev_detection / bev_tracking / bev_interfaces /
  m4_demo_bringup 一致。代码 license 与 checkpoint license 是两件事。
- 文档与脚本 (`docs/`, `scripts/m4/`) 沿用本仓库现有 license。

## Legal Note
- 课程文档**严禁**声明该 checkpoint 是 MIT 或 commercial-friendly。
  上述 release gate 已完成，结论是 **terms unspecified**，因此限制更严而非更松。
- 任何 P0 完成声明必须显式包含 "License verified before release" 语句。
