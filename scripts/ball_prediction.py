#!/usr/bin/env python3
#
# Copyright 2026 RFC-Tsudanuma
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the “Software”), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# [Contributors]
#
# - Masafumi Horiguchi
# - Satoshi Inoue

from abc import ABC, abstractmethod
import numpy as np
from numpy.typing import NDArray
from setting import STRATEGY_SHARED_CONFIG

class PredictBallABC(ABC):
    def __init__(self,alpha=0.5):
        super().__init__()
        self.alpha = alpha
        self.y_prev = 0
        self.y_obs = 0

    @abstractmethod
    def predict(self, x_t:NDArray[np.float64]) -> NDArray[np.float64]:
        """ボールの位置を予測するシステムを構成する
        :param x_t: (x,y): 現在のボールのグローバル座標
        :return: 予測されたボールのグローバル座標
        """
        pass

    @abstractmethod
    def predict_for_debug(self,x_t:NDArray[np.float64]) -> NDArray[np.float64]:
        pass

    def lpf_pos(self,filterd_position:NDArray[np.float64]) -> NDArray[np.float64]:
        """
        lpfの実装
        """
        self.y_prev = self.alpha * filterd_position + (1 - self.alpha) * self.y_prev
        return self.y_prev
    
    def lpf_obs(self,filterd_position:NDArray[np.float64]) -> NDArray[np.float64]:
        """
        lpfの実装
        """
        self.y_obs = self.alpha * filterd_position + (1 - self.alpha) * self.y_obs
        return self.y_obs
    
    

class SamplingPredict(PredictBallABC):
    """サンプリングによってボールの軌道を予測するクラスの親"""
    def __init__(self, particle_num=100,alpha=0.5,num_seq:int=2):
        """
        :input: particle_num    使用するパーティクルの数
                alpha           lpfの反映ゲイン
                num_seq         保持するデータ量
        num_seqを増やした場合ノイズに強くなる代わりに応答性が下がる
        """
        super().__init__(alpha=alpha)
        self.particle_num = particle_num
        self.sampled = np.zeros((particle_num,2))
        self.x_tm = np.zeros((num_seq,2)) # x_(t~t-num_seq)
        # self.x_tm1 = np.zeros(2) # x_(t-1)
        self.missing_count = 0
        self.num_seq = num_seq

        self.sigma = 1.0

        self.weights = self.filter((particle_num,2))
        self.filtered_position = np.zeros(2)
        self.dxs = np.zeros((num_seq-1,2))

    def shift(self,val:NDArray,vals:NDArray) -> None:
        vals[1:] = vals[:-1]
        vals[0] = val

    def init_values(self,x_t:NDArray[np.float64]) -> None:
        """初期値を設定する"""
        # ベクトル化: forループを削除
        self.x_tm[:self.num_seq-1] = x_t
        self.dxs[:] = 0
        self.filtered_position = x_t
        self.weights = self.filter(x_t)

    def predict(self, x_t:NDArray[np.float64]) -> NDArray[np.float64]:
        """サンプリングアルゴリズムによりボールのパーティクルを生成する
        :param x_t: (x,y): 現在のボールのグローバル座標
        :return: 予測されたボールのパーティクル
        """
        if x_t is None or  np.isnan(x_t).all():
            self.missing_count += 1

            # ベクトル化: sampling_batch()を使用
            noise = self.sampling_batch(self.filtered_position, self.particle_num)
            self.sampled[:] = self.filtered_position + noise
            # 見失ってからも予測値を信用し続けると値が吹っ飛んでいくので値の更新をやめる
            if self.missing_count > STRATEGY_SHARED_CONFIG["filter_config"]["pf_config"]["missing_count"]:
                return None
            self.filtered_position = np.sum(self.sampled * self.weights[:,np.newaxis],axis=0)
            self.resample(self.weights)

            return self.lpf_pos(self.filtered_position)

        if self.missing_count > 10:
            self.init_values(x_t)
        self.missing_count = 0
        self.weights = self.filter(x_t)
        
        """
        参考にした実装では自己位置推定に使っていたため,現在の観測にノイズが乗ることを考慮した実装になるはず
        今回はボールの予測に使うため、現在の観測値を未来の状態に反映させる方針で行く
        具体的には、従来方針ではサンプルを更新後、観測値とサンプリングされたパーティクルの差分から重みを計算しているのに対し
        今回方法ではサンプルを更新前に観測値x_tとサンプリングされたパーティクルの差分から重みを計算する
        これにより、前回の予測誤差から未来の状態の尤度を計算してフィルターをかけることができる
        
        おそらく、x_{t+1}の予測はx_{1:t}によって決定できるため、従来設計と同じ役割を果たせるはず
        """
        self.shift(x_t,self.x_tm)
        self.shift(self.x_tm[0]-self.x_tm[1],self.dxs)
        # ベクトル化: sampling_batch()を使用してパーティクルを一度に生成
        noise = self.sampling_batch(x_t, self.particle_num)
        self.sampled[:] = x_t + noise  # t+1 の予測値のパーティクル

        self.filtered_position = np.sum(self.sampled * self.weights[:, np.newaxis], axis=0)

        self.resample(self.weights)

        # self.x_tm1 = x_t # t+1の予測のときに使うために保存

        return self.lpf_pos(self.filtered_position) # 予測値はサンプリングの平均

    def predict_for_debug(self, x_t:NDArray[np.float64]) -> NDArray[np.float64]:
        """サンプリングアルゴリズムによりボールのパーティクルを生成する（デバッグ用）
        :param x_t: (x,y): 現在のボールのグローバル座標
        :return: 予測されたボールのパーティクル
        """
        filtered_position = self.predict(x_t)
        return filtered_position,self.sampled # 予測値はサンプリングの値
    
    @abstractmethod
    def filter(self,x_t:NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """フィルターを実装する
        :param x_t: (x,y): 現在のボールのグローバル座標(観測値)
        :return filtered_position: フィルターされたボールのグローバル座標
                weights: 各パーティクルの重み
        """
        pass

    @abstractmethod
    def sampling(self, x:NDArray[np.float64]) -> NDArray[np.float64]:
        """サンプリングアルゴリズムを実装する
        :param x: (x,y): ボールの位置
        :return: サンプリングされたノイズ
        """
        pass

    def sampling_batch(self, x:NDArray[np.float64], batch_size:int) -> NDArray[np.float64]:
        """複数のノイズを一度にサンプリングする（デフォルト実装）
        子クラスでオーバーライドして最適化可能
        :param x: (x,y): ボールの位置
        :param batch_size: 生成するサンプル数
        :return: サンプリングされたノイズの配列 (batch_size, 2)
        """
        # デフォルトはforループで実装（子クラスでベクトル化版をオーバーライド推奨）
        return np.array([self.sampling(x) for _ in range(batch_size)])

    @abstractmethod
    def resample(self,weights:NDArray[np.float64]) -> None:
        """リサンプリングを実装する
        :param weights: 各パーティクルの重み
        :return: None
        """
        pass


class NormalSamplingPredict(SamplingPredict):
    def __init__(self, particle_num=100,alpha=0.5,randamize_param=12,num_seq:int=2):
        """近似された正規分布に従うノイズをサンプリングする"""
        super().__init__(particle_num,alpha=alpha,num_seq=num_seq)
        self.randamize_param = randamize_param

    def filter(self, x_t:NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        pass # フィルターは実装しない

    def sampling(self, x:NDArray[np.float64]) -> NDArray[np.float64]:
        """近似された正規分布に従うノイズをサンプリングする
        :param x: (x,y): ボールの位置
        :return: サンプリングされたノイズ
        """
        sample = self.dxs[0].copy()  # copy()はここで必要
        scale_factor = 1 + 0.001 * self.missing_count

        abs_dxs = np.clip(np.abs(self.dxs), 0, 100) * scale_factor
        # ベクトル化: forループを削除して一度にサンプリング
        sample += np.mean(np.random.uniform(-abs_dxs, abs_dxs,
                    size=(self.randamize_param, self.num_seq-1, 2)), axis=(0, 1))
        return sample * 0.5

    def sampling_batch(self, x:NDArray[np.float64], batch_size:int) -> NDArray[np.float64]:
        """複数のノイズを一度にサンプリングする（高速化版）
        :param x: (x,y): ボールの位置
        :param batch_size: 生成するサンプル数
        :return: サンプリングされたノイズの配列 (batch_size, 2)
        """
        sample_base = self.dxs[0]
        scale_factor = 1 + 0.001 * self.missing_count

        abs_dxs = np.clip(np.abs(self.dxs), 0, 100) * scale_factor
        # (batch_size, randamize_param, num_seq-1, 2) の形状でサンプリング
        random_samples = np.random.uniform(-abs_dxs, abs_dxs,
                                          size=(batch_size, self.randamize_param, self.num_seq-1, 2))
        # 各バッチで平均を取る: (batch_size, 2)
        samples = sample_base + np.mean(random_samples, axis=(1, 2))
        return samples * 0.5

    def resample(self,weights:NDArray[np.float64]) -> NDArray:
        pass # リサンプリングは実装しない

class NormalSamplingPredictWithParticleGausianFilter(NormalSamplingPredict):
    def __init__(self, particle_num=100,alpha=0.5,randamize_param=12,num_seq:int=2):
        """近似された正規分布に従うノイズをサンプリングし、パーティクルフィルターを実装する"""
        super().__init__(particle_num, alpha,randamize_param,num_seq)

    def filter(self, x_t:NDArray[np.float64]) -> NDArray[np.float64]:
        """パーティクルフィルターを実装する
        :param x_t: (x,y): 現在のボールのグローバル座標(観測値)
        :return: フィルターされたボールのグローバル座標
        """
        diff = x_t - self.sampled
        # 最適化: 除算を減らし、sigma^2を事前計算
        sigma_sq = self.sigma * self.sigma
        weights = np.exp(-0.5 * np.sum(diff * diff, axis=1) / sigma_sq) + 1e-10  # shape: (particle_num,)
        weights /= np.sum(weights)  # 正規化
        return weights

    def sampling(self, x:NDArray[np.float64]) -> NDArray[np.float64]:
        return super().sampling(x)
    
    def resample(self, weights)->None:
        indices = np.random.choice(self.particle_num, size=self.particle_num, p=weights)
        self.sampled = self.sampled[indices]

class RegressionPredict(PredictBallABC):
    """回帰分析によってボールの軌道を予測するクラスの親"""
    def __init__(self,num_sample:int,alpha):
        super().__init__(alpha=alpha)
        self.samples = np.zeros((num_sample,2))
        self.numsample = num_sample

    def predict(self, x_t):
        self.update_sample(self.samples,x_t)
        nex_pos = self.method()
        return nex_pos
    
    def predict_for_debug(self, x_t):
        return self.predict(x_t)
    
    def update_sample(self,samples:NDArray[np.float64],new_data:NDArray[np.float64]) -> None:
        """使うサンプルをアップデートする関数"""
        samples[:] = np.roll(samples,-1,axis=0)
        samples[-1] = new_data

    @abstractmethod
    def method(self) -> NDArray[np.float64]:
        """アルゴリズムを実装する
        :return (x,y): 予測位置
        """
        pass

class RegressionPredictWithLeatestSquaresMethod(RegressionPredict):
    """最小二乗法によりボールの軌道を予測する"""
    def __init__(self, num_sample,alpha):
        super().__init__(num_sample,alpha=alpha)
        self.diffs = np.zeros(num_sample-1)

    def method(self):
        """
        次の式より軌跡を予測
        a = (nΣxy - ΣxΣy) / (nΣx^2 - (Σx)^2)
        b = (Σy - aΣx) / n
        また、dxを予測して次の時点の座標を予測
        """
        n = self.numsample
        x = self.samples[:,0]
        y = self.samples[:,1]

        # 最小二乗法の係数 a, b
        a = (n*np.sum(x*y) - np.sum(x)*np.sum(y)) / (n*np.sum(x**2) - np.sum(x)**2)
        b = (np.sum(y) - a*np.sum(x)) / n

        # xの移動量の平均
        last_diff = x[-1] - x[-2]
        self.diffs[:] = np.roll(self.diffs, -1)
        self.diffs[-1] = last_diff

        # 予測
        nex_x = x[-1] + np.mean(self.diffs)
        nex_y = a * nex_x + b

        # print(np.array([nex_x,nex_y]))
        
        return self.lpf_pos(np.array([nex_x, nex_y]))
    
    


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    # predictor = RegressionPredictWithLeatestSquaresMethod(num_sample=3,alpha=0.5)
    predictor = NormalSamplingPredictWithParticleGausianFilter(particle_num=200, randamize_param=12)

    scale = 1
    # ===== シミュレーション設定 =====
    step_num = 50                        # ステップ数
    x_tm1 = np.array([0., 0.])             # 初期位置
    v = np.array([0., 0.])                 # 速度ベクトル
    predictor.x_tm1 = x_tm1

    # ノイズ入り観測データ生成
    obs_positions = []
    for t in range(1, step_num+1):
        # 本当の位置
        true_pos = x_tm1 + v * t
        # 観測ノイズを付加
        noise = np.random.normal(0, 500, size=2)
        x_obs = true_pos + noise
        
        if np.random.random(1) > 0.6:
            x_obs = [np.nan,np.nan]
            print("LOST",t)
        obs_positions.append(x_obs)

    obs_positions = np.array(obs_positions) / scale

    # ===== 観測を欠損させる =====
    # 例: ステップ5〜8を観測なしにする
    obs_true = np.copy(obs_positions)

    # ===== 予測 =====
    predicted_positions = []
    all_particles = []


    for t in range(step_num):
        filtered_position, particles = predictor.predict_for_debug(obs_positions[t])
        predicted_positions.append(filtered_position)
        all_particles.append(particles.copy())  # パーティクルの分布も保存

    predicted_positions = np.array(predicted_positions) * scale

    # ===== 可視化 =====
    plt.figure(figsize=(10, 6))

    # 観測値（赤）、欠損（青×）
    for t, pos in enumerate(obs_positions*scale):
        if pos is None or np.isnan(pos).all():
            # print(true_pos)
            plt.plot(obs_true[t][0],obs_true[t][1],'x',color='blue')
        plt.plot(pos[0], pos[1], 'o', color='red')

    # 予測値（緑）
    plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 'x-', color='green', label='Prediction')

    # パーティクル分布（薄いオレンジ）
    for t, particles in enumerate(all_particles):
        plt.scatter(particles[:, 0], particles[:, 1], color='orange', alpha=0.1)

    plt.axis('equal')
    plt.legend()
    plt.title("Particle Filter Ball Tracking (Multi-step)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

    print("Final predicted position:", predicted_positions[-1])
