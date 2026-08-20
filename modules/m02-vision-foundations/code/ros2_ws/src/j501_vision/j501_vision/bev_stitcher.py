#!/usr/bin/env python3
"""多相机 BEV 拼接 - 加权融合"""

import cv2
import numpy as np

class BEVStitcher:
    """多相机 BEV 拼接器"""

    def __init__(self, bev_width=1000, bev_height=1000):
        """初始化 BEV 拼接器

        Args:
            bev_width: BEV 图像宽度
            bev_height: BEV 图像高度
        """
        self.bev_width = bev_width
        self.bev_height = bev_height

        # 存储各相机的 BEV 图像和权重图
        self.bev_images = {}
        self.weight_maps = {}

    def set_camera_bev(self, cam_name, bev_image, weight_map=None):
        """设置单相机的 BEV 图像

        Args:
            cam_name: 相机名称（'front', 'left', 'right', 'rear'）
            bev_image: BEV 图像
            weight_map: 权重图（0-1），None 则自动生成
        """
        self.bev_images[cam_name] = bev_image

        if weight_map is None:
            # 自动生成距离权重图（中心权重高）
            weight_map = self._create_distance_weight(bev_image.shape)
        self.weight_maps[cam_name] = weight_map

    def _create_distance_weight(self, shape):
        """创建距离权重图

        距离相机越近（BEV 中心）权重越高

        Args:
            shape: 图像尺寸 (height, width)

        Returns:
            权重图（0-1）
        """
        h, w = shape[:2]
        # 创建径向距离图
        y, x = np.ogrid[:h, :w]
        center_y, center_x = h / 2, w / 2
        dist = np.sqrt((y - center_y)**2 + (x - center_x)**2)
        max_dist = np.sqrt(center_y**2 + center_x**2)
        # 线性衰减
        weight = 1.0 - (dist / max_dist) * 0.5
        weight = np.clip(weight, 0, 1)
        return weight.astype(np.float32)

    def _create_feather_weight(self, shape, direction='center'):
        """创建羽化权重图

        在图像边缘渐变到 0，避免拼接接缝

        Args:
            shape: 图像尺寸
            direction: 羽化方向

        Returns:
            权重图
        """
        h, w = shape[:2]
        feather_size = min(h, w) // 10  # 羽化区域大小

        weight = np.ones((h, w), dtype=np.float32)

        # 边缘羽化
        for i in range(feather_size):
            alpha = i / feather_size
            weight[i, :] = min(weight[i, :], alpha)
            weight[h-1-i, :] = min(weight[h-1-i, :], alpha)
            weight[:, i] = min(weight[:, i], alpha)
            weight[:, w-1-i] = min(weight[:, w-1-i], alpha)

        return weight

    def stitch_weighted(self):
        """加权融合拼接

        Returns:
            拼接后的 BEV 图像
        """
        if not self.bev_images:
            return None

        # 初始化累加器
        result = np.zeros(
            (self.bev_height, self.bev_width, 3), dtype=np.float32
        )
        weight_sum = np.zeros(
            (self.bev_height, self.bev_width), dtype=np.float32
        )

        # 累加各相机贡献
        for cam_name in self.bev_images:
            bev = self.bev_images[cam_name].astype(np.float32)
            weight = self.weight_maps[cam_name]

            # 确保尺寸匹配
            if bev.shape[:2] != (self.bev_height, self.bev_width):
                bev = cv2.resize(bev, (self.bev_width, self.bev_height))
                weight = cv2.resize(
                    weight, (self.bev_width, self.bev_height)
                )

            # 广播权重到 3 通道
            weight_3c = np.stack([weight] * 3, axis=-1)
            result += bev * weight_3c
            weight_sum += weight

        # 归一化
        weight_sum = np.maximum(weight_sum, 1e-6)  # 避免除零
        result = result / np.stack([weight_sum] * 3, axis=-1)

        return result.astype(np.uint8)

    def stitch_max(self):
        """最大值拼接（选择每个像素最亮的相机）

        Returns:
            拼接后的 BEV 图像
        """
        if not self.bev_images:
            return None

        result = np.zeros(
            (self.bev_height, self.bev_width, 3), dtype=np.uint8
        )

        for cam_name in self.bev_images:
            bev = self.bev_images[cam_name]
            if bev.shape[:2] != (self.bev_height, self.bev_width):
                bev = cv2.resize(bev, (self.bev_width, self.bev_height))
            result = np.maximum(result, bev)

        return result


if __name__ == '__main__':
    stitcher = BEVStitcher(bev_width=1000, bev_height=1000)

    # 模拟 4 路相机 BEV 图像
    for cam_name in ['front', 'left', 'right', 'rear']:
        # 随机生成模拟 BEV 图像
        bev = np.random.randint(
            0, 255, (1000, 1000, 3), dtype=np.uint8
        )
        stitcher.set_camera_bev(cam_name, bev)

    # 加权融合
    result = stitcher.stitch_weighted()
    cv2.imwrite('/tmp/bev_stitched.jpg', result)
    print("BEV 拼接图已保存到 /tmp/bev_stitched.jpg")


#!/usr/bin/env python3
"""使用 OpenCV 的多频段融合进行 BEV 拼接"""

import cv2
import numpy as np

class MultiBandBlender:
    """多频段融合器"""

    def __init__(self, num_bands=5):
        """初始化多频段融合器

        Args:
            num_bands: 金字塔层数
        """
        self.num_bands = num_bands

    def blend(self, img1, mask1, img2, mask2):
        """融合两幅图像

        Args:
            img1: 图像 1
            mask1: 图像 1 的权重图（0-1）
            img2: 图像 2
            mask2: 图像 2 的权重图（0-1）

        Returns:
            融合后的图像
        """
        # 确保图像为 float32
        img1 = img1.astype(np.float32)
        img2 = img2.astype(np.float32)
        mask1 = mask1.astype(np.float32)
        mask2 = mask2.astype(np.float32)

        # 构建拉普拉斯金字塔
        lp1 = self._build_laplacian_pyramid(img1, self.num_bands)
        lp2 = self._build_laplacian_pyramid(img2, self.num_bands)
        gp1 = self._build_gaussian_pyramid(mask1, self.num_bands)
        gp2 = self._build_gaussian_pyramid(mask2, self.num_bands)

        # 在每层融合
        blended = []
        for l1, l2, g1, g2 in zip(lp1, lp2, gp1, gp2):
            # 广播权重到 3 通道
            g1_3c = np.stack([g1] * 3, axis=-1) if len(g1.shape) == 2 else g1
            g2_3c = np.stack([g2] * 3, axis=-1) if len(g2.shape) == 2 else g2
            blended.append(l1 * g1_3c + l2 * g2_3c)

        # 金字塔重建
        result = blended[-1]
        for i in range(len(blended) - 2, -1, -1):
            h, w = blended[i].shape[:2]
            result = cv2.pyrUp(result, dstsize=(w, h))
            result = result + blended[i]

        return np.clip(result, 0, 255).astype(np.uint8)

    def _build_laplacian_pyramid(self, image, levels):
        """构建拉普拉斯金字塔"""
        gaussian = image.copy()
        lp = []
        for i in range(levels):
            h, w = gaussian.shape[:2]
            down = cv2.pyrDown(gaussian)
            up = cv2.pyrUp(down, dstsize=(w, h))
            lap = gaussian - up
            lp.append(lap)
            gaussian = down
        lp.append(gaussian)  # 最后一层是低频残差
        return lp

    def _build_gaussian_pyramid(self, mask, levels):
        """构建高斯金字塔"""
        gp = [mask]
        for i in range(levels):
            mask = cv2.pyrDown(mask)
            gp.append(mask)
        return gp


if __name__ == '__main__':
    blender = MultiBandBlender(num_bands=5)

    # 模拟两幅图像
    img1 = np.random.randint(100, 200, (512, 512, 3), dtype=np.uint8)
    img2 = np.random.randint(150, 250, (512, 512, 3), dtype=np.uint8)

    # 创建权重图（上半给 img1，下半给 img2，中间 100 行过渡）
    mask1 = np.zeros((512, 512), dtype=np.float32)
    mask2 = np.zeros((512, 512), dtype=np.float32)
    for i in range(512):
        alpha = max(0, min(1, (256 - i) / 100))
        mask1[i, :] = alpha
        mask2[i, :] = 1 - alpha

    result = blender.blend(img1, mask1, img2, mask2)
    cv2.imwrite('/tmp/bev_blended.jpg', result)
    print("多频段融合结果已保存到 /tmp/bev_blended.jpg")
