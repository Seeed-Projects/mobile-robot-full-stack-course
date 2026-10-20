# Sources and Asset Terms

[English](THIRD_PARTY.md) | [中文](THIRD_PARTY_zh_CN.md)

- `overlay/` uses the fixed LightNav-0 commit as its baseline and includes this project's deployment and chassis-simulation adaptations. Copies of the upstream [license](licenses/LightNav-LICENSE) and [third-party notices](licenses/LightNav-THIRD_PARTY_NOTICES.md) are retained. The complete notices do not imply that this package contains every named component.
- Upstream ProcTHOR/MolmoSpaces scenes have their own asset and attribution terms. The code license does not automatically cover scene assets, and the course does not copy the entire dataset.
- CAD comes from the user-supplied Onshape assembly export, with original export SHA256, simplified-mesh inventory, and parameter sources preserved. A separate instructor bundle supports reproduction; this does not establish public redistribution permission.
- NVIDIA/Seeed wheels, cuSPARSELt, and Hugging Face weights come from their release channels. The course supplies versions, checksums, and preparation instructions.
- Course tools prepare materials and analyze records offline; they do not access a physical SDK. `sdk-chassis.json` is frozen geometry/direction configuration, without a device session, password, or token.
- This work does not change the repository's still-unsettled overall licensing choice or assign a single license to all teaching content, CAD, and third-party code.
