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
# - Satoshi Inoue
# - Suzuha Kiuchi


import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
from state import Position
from numba import njit
from time import perf_counter

from print_utils import print_in_strategy

def generate_clearance_circle_nodes(
    obstacles: np.ndarray, clearance: float, num_nodes_per_obstacle: int = 8
):
    """
    各障害物のクリアランス円周上にノードを生成（完全ベクトル化版）

    :param obstacles: 障害物のndarray (shape: (n, 2) where n is number of obstacles)
    :param clearance: クリアランス半径
    :param num_nodes_per_obstacle: 各障害物あたりのノード数
    :return: ノード座標のリスト、各ノードがどの障害物に属するかのリスト
    """
    num_obstacles = len(obstacles)
    if num_obstacles == 0:
        return [], []

    # 角度を事前計算 (M,)
    angles = np.linspace(0, 2 * np.pi, num_nodes_per_obstacle, endpoint=False)

    # 三角関数を事前計算
    cos_angles = np.cos(angles)
    sin_angles = np.sin(angles)

    # 全ノード候補を一度に生成
    # obstacles: (N, 2) -> (N, 1, 2)
    # [cos, sin]: (M, 2)
    # 結果: (N, M, 2)
    obstacles_expanded = obstacles[:, np.newaxis, :]  # (N, 1, 2)
    offsets = clearance * np.stack([cos_angles, sin_angles], axis=1)  # (M, 2)

    # ブロードキャストで全候補ノードを生成 (N, M, 2)
    all_candidates = obstacles_expanded + offsets[np.newaxis, :, :]

    # (N, M, 2) -> (N*M, 2) にreshape
    all_candidates_flat = all_candidates.reshape(-1, 2)

    # 各候補がどの障害物に属するか (N*M,)
    candidate_obs_indices = np.repeat(np.arange(num_obstacles), num_nodes_per_obstacle)

    # 全候補ノードと全障害物の距離を一度に計算
    # all_candidates_flat: (N*M, 2), obstacles: (N, 2)
    # distances_sq: (N*M, N)
    diff = all_candidates_flat[:, np.newaxis, :] - obstacles[np.newaxis, :, :]  # (N*M, N, 2)
    distances_sq = np.sum(diff ** 2, axis=2)  # (N*M, N)

    # 自分自身の障害物は除外するマスク
    self_mask = candidate_obs_indices[:, np.newaxis] == np.arange(num_obstacles)[np.newaxis, :]  # (N*M, N)

    # 自分以外の障害物との距離が閾値以上かチェック
    threshold_sq = (clearance - 1e-6) ** 2
    distances_sq_masked = np.where(self_mask, np.inf, distances_sq)  # 自分自身は無限大にして除外

    # 全ての他の障害物との距離が閾値以上ならvalid
    valid_mask = np.all(distances_sq_masked >= threshold_sq, axis=1)  # (N*M,)

    # 有効なノードのみを抽出
    valid_nodes = all_candidates_flat[valid_mask]
    valid_obs_indices = candidate_obs_indices[valid_mask]

    # リスト形式で返す（互換性のため）
    nodes = [valid_nodes[i] for i in range(len(valid_nodes))]
    node_obstacle_map = [int(valid_obs_indices[i]) for i in range(len(valid_obs_indices))]

    return nodes, node_obstacle_map

def add_tangent_nodes_for_point(point, obstacles, clearance, nodes, node_obstacle_map):
    """
    指定した点から各障害物への接線の接点をノードとして追加（ベクトル化版）
    """
    if len(obstacles) == 0:
        return nodes, node_obstacle_map

    # 点から全障害物への距離を一度に計算
    # obstacles: (N, 2), point: (2,)
    diff = obstacles - point[np.newaxis, :]  # (N, 2)
    distances = np.linalg.norm(diff, axis=1)  # (N,)

    # 点が円の外部にある障害物のみ処理（内部なら接線なし）
    valid_obstacles_mask = distances > clearance  # (N,)
    valid_indices = np.where(valid_obstacles_mask)[0]

    if len(valid_indices) == 0:
        return nodes, node_obstacle_map

    # 有効な障害物のみ抽出
    valid_obstacles = obstacles[valid_indices]  # (M, 2)
    valid_distances = distances[valid_indices]  # (M,)
    valid_diff = diff[valid_indices]  # (M, 2)

    # 接線の角度を一度に計算
    angles = np.arcsin(clearance / valid_distances)  # (M,)
    cos_angles = np.cos(angles)  # (M,)
    sin_angles = np.sin(angles)  # (M,)

    # 正規化されたベクトル
    unit_vecs = valid_diff / valid_distances[:, np.newaxis]  # (M, 2)

    # 2つの接点への方向ベクトルを一度に計算
    # dir1 = R(-angle) @ unit_vec
    # dir2 = R(+angle) @ unit_vec
    dir1_x = cos_angles * unit_vecs[:, 0] - sin_angles * unit_vecs[:, 1]  # (M,)
    dir1_y = sin_angles * unit_vecs[:, 0] + cos_angles * unit_vecs[:, 1]  # (M,)
    dir2_x = cos_angles * unit_vecs[:, 0] + sin_angles * unit_vecs[:, 1]  # (M,)
    dir2_y = -sin_angles * unit_vecs[:, 0] + cos_angles * unit_vecs[:, 1]  # (M,)

    # 接点の座標を一度に計算
    tangent_points1 = valid_obstacles + clearance * np.stack([dir1_x, dir1_y], axis=1)  # (M, 2)
    tangent_points2 = valid_obstacles + clearance * np.stack([dir2_x, dir2_y], axis=1)  # (M, 2)

    # 2つの接点グループを統合 (2*M, 2)
    all_tangent_points = np.vstack([tangent_points1, tangent_points2])  # (2*M, 2)
    tangent_obs_indices = np.concatenate([valid_indices, valid_indices])  # (2*M,)

    # 全接点と全障害物の距離を一度に計算
    # all_tangent_points: (2*M, 2), obstacles: (N, 2)
    diff_all = all_tangent_points[:, np.newaxis, :] - obstacles[np.newaxis, :, :]  # (2*M, N, 2)
    distances_all_sq = np.sum(diff_all ** 2, axis=2)  # (2*M, N)

    # 自分自身の障害物は除外するマスク
    self_mask = tangent_obs_indices[:, np.newaxis] == np.arange(len(obstacles))[np.newaxis, :]  # (2*M, N)

    # 自分以外の障害物との距離が閾値以上かチェック
    threshold_sq = (clearance - 1e-6) ** 2
    distances_all_sq_masked = np.where(self_mask, np.inf, distances_all_sq)

    # 全ての他の障害物との距離が閾値以上ならvalid
    valid_tangents_mask = np.all(distances_all_sq_masked >= threshold_sq, axis=1)  # (2*M,)

    # 有効な接点のみを抽出して追加
    valid_tangent_points = all_tangent_points[valid_tangents_mask]
    valid_tangent_obs_indices = tangent_obs_indices[valid_tangents_mask]

    for i in range(len(valid_tangent_points)):
        nodes.append(valid_tangent_points[i])
        node_obstacle_map.append(int(valid_tangent_obs_indices[i]))

    return nodes, node_obstacle_map

def check_lines_circles_intersection_vectorized(line_starts, line_ends, circles, radius):
    """
    複数の線分と複数の円の交差判定を一括で行う（ベクトル化版）

    Args:
        line_starts: (N_edges, 2) 線分の始点
        line_ends: (N_edges, 2) 線分の終点
        circles: (N_obstacles, 2) 円の中心
        radius: float クリアランス半径

    Returns:
        (N_edges, N_obstacles) の bool 配列。True = 交差あり
    """
    # 線分の始点・終点が円の内部にあるかチェック
    # line_starts: (N_edges, 2), circles: (N_obstacles, 2)
    # -> (N_edges, N_obstacles)
    dist_start = np.linalg.norm(line_starts[:, np.newaxis, :] - circles[np.newaxis, :, :], axis=2)
    dist_end = np.linalg.norm(line_ends[:, np.newaxis, :] - circles[np.newaxis, :, :], axis=2)

    inside_check = (dist_start < radius) | (dist_end < radius)  # (N_edges, N_obstacles)

    # 線分のベクトル d = end - start
    d = line_ends - line_starts  # (N_edges, 2)

    # f = start - circle_center
    # (N_edges, 1, 2) - (1, N_obstacles, 2) = (N_edges, N_obstacles, 2)
    f = line_starts[:, np.newaxis, :] - circles[np.newaxis, :, :]

    # a = d・d (各エッジごと)
    a = np.sum(d * d, axis=1, keepdims=True)  # (N_edges, 1)

    # 長さ0のエッジ（始点=終点）の場合はinside_checkのみで判定
    zero_length_edges = (a < 1e-10)  # (N_edges, 1)

    # b = 2 * f・d (各エッジと各障害物の組み合わせ)
    b = 2 * np.sum(f * d[:, np.newaxis, :], axis=2)  # (N_edges, N_obstacles)

    # c = f・f - r²
    c = np.sum(f * f, axis=2) - radius**2  # (N_edges, N_obstacles)

    # 判別式
    discriminant = b**2 - 4 * a * c  # (N_edges, N_obstacles)

    # 判別式が負なら交点なし
    no_intersection = discriminant < 0

    # ゼロ除算を避けるため、aが小さい場合は安全な値に置き換え
    a_safe = np.where(a < 1e-10, 1.0, a)  # (N_edges, 1)

    # tが0-1の範囲にあるかチェック
    sqrt_discriminant = np.sqrt(np.maximum(discriminant, 0))  # 負の値は0にクリップ
    t1 = (-b - sqrt_discriminant) / (2 * a_safe)  # (N_edges, N_obstacles)
    t2 = (-b + sqrt_discriminant) / (2 * a_safe)  # (N_edges, N_obstacles)

    # t1またはt2が0-1の範囲にあるか
    t_in_range = ((0 <= t1) & (t1 <= 1)) | ((0 <= t2) & (t2 <= 1))

    # 線分が円を完全に含む場合
    contains_circle = (t1 < 0) & (t2 > 1)

    # 最終的な交差判定（長さ0のエッジは除外）
    intersects = inside_check | (np.where(zero_length_edges, False, ~no_intersection & (t_in_range | contains_circle)))

    return intersects  # (N_edges, N_obstacles)

def build_graph(
    nodes, obstacles, clearance, clearance_factor, start_pos=None, goal_pos=None
):
    """
    ノード間のグラフを構築（完全ベクトル化版）
    """
    num_nodes = len(nodes)

    # start/goalを含めた全ノードリスト
    all_nodes = nodes.copy()
    if start_pos is not None:
        all_nodes.insert(0, start_pos)  # index 0 が start
        num_nodes += 1
    if goal_pos is not None:
        all_nodes.append(goal_pos)  # 最後が goal
        num_nodes += 1

    # 全ノードを配列に変換
    all_nodes_array = np.array(all_nodes)  # (num_nodes, 2)

    # 全エッジのペアのインデックスを生成
    edge_indices = np.array([(i, j) for i in range(num_nodes) for j in range(i + 1, num_nodes)])
    total_edges_checked = len(edge_indices)

    if total_edges_checked == 0:
        return np.zeros((num_nodes, num_nodes)), all_nodes

    # 全エッジの始点と終点を取得
    edge_starts = all_nodes_array[edge_indices[:, 0]]  # (N_edges, 2)
    edge_ends = all_nodes_array[edge_indices[:, 1]]  # (N_edges, 2)

    if len(obstacles) > 0:
        # バウンディングボックスチェック（早期カット）
        edge_centers = (edge_starts + edge_ends) / 2  # (N_edges, 2)
        edge_half_lengths = np.linalg.norm(edge_ends - edge_starts, axis=1) / 2  # (N_edges,)

        # 各エッジ中点から各障害物への距離
        dist_to_obstacles = np.linalg.norm(
            edge_centers[:, np.newaxis, :] - obstacles[np.newaxis, :, :], axis=2
        )  # (N_edges, N_obstacles)

        # バウンディングボックス判定
        bbox_threshold = edge_half_lengths[:, np.newaxis] + clearance * clearance_factor
        potential_collision = dist_to_obstacles <= bbox_threshold  # (N_edges, N_obstacles)

        # 詳細な交差判定（バウンディングボックスで候補になったもののみ）
        intersections = check_lines_circles_intersection_vectorized(
            edge_starts, edge_ends, obstacles, clearance * clearance_factor
        )  # (N_edges, N_obstacles)

        # バウンディングボックスで除外されたものは交差なしとする
        intersections = intersections & potential_collision

        # 各エッジについて、いずれかの障害物と交差していればNG
        edge_valid = ~np.any(intersections, axis=1)  # (N_edges,)
    else:
        # 障害物がなければ全エッジ有効
        edge_valid = np.ones(total_edges_checked, dtype=bool)

    # エッジの距離を計算
    edge_distances = np.linalg.norm(edge_ends - edge_starts, axis=1)  # (N_edges,)

    # 隣接行列を構築
    adjacency = np.zeros((num_nodes, num_nodes))

    # 有効なエッジのみを抽出
    valid_edge_indices = edge_indices[edge_valid]
    valid_edge_distances = edge_distances[edge_valid]

    # ベクトル化された代入
    adjacency[valid_edge_indices[:, 0], valid_edge_indices[:, 1]] = valid_edge_distances
    adjacency[valid_edge_indices[:, 1], valid_edge_indices[:, 0]] = valid_edge_distances

    valid_edges_count = len(valid_edge_indices)

    print_in_strategy(f"<Pathplan> Graph construction: {valid_edges_count}/{total_edges_checked} edges are valid")

    return adjacency, all_nodes


def find_shortest_path(adjacency, all_nodes, start_idx=0, goal_idx=-1):
    """
    グラフ上で最短経路を探索
    """
    if goal_idx < 0:
        goal_idx = len(all_nodes) + goal_idx

    # Dijkstra法で最短経路を計算
    graph = csr_matrix(adjacency)
    dist_matrix, predecessors = shortest_path(
        graph, directed=False, indices=start_idx, return_predecessors=True
    )

    if np.isinf(dist_matrix[goal_idx]):
        print_in_strategy("<Pathplan> No path found.")
        return [], float("inf")

    # 経路を復元
    path_indices = []
    current = goal_idx
    while current != start_idx and current != -9999:
        path_indices.append(current)
        current = predecessors[current]
    path_indices.append(start_idx)
    path_indices.reverse()

    # ノード座標のリストに変換
    path_nodes = [all_nodes[idx] for idx in path_indices]

    # 総距離を計算
    total_distance = dist_matrix[goal_idx]

    return path_nodes, total_distance

@njit(cache=True)
def check_edge_validity_simple(p1: np.ndarray, p2: np.ndarray, obstacles: np.ndarray, clearance: float):
    """
    エッジがクリアランス円と交差しないかチェック（smooth_path用の簡易版）
    """
    if len(obstacles) == 0:
        return True

    # 線分のベクトル
    d = p2 - p1
    a = np.dot(d, d)

    for i in range(len(obstacles)):
        obstacle_pos = obstacles[i]

        # 線分の始点・終点が円の内部にある場合は交差とみなす
        if np.linalg.norm(p1 - obstacle_pos) < clearance or np.linalg.norm(p2 - obstacle_pos) < clearance:
            return False

        # f = start - circle_center
        f = p1 - obstacle_pos

        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - clearance**2

        discriminant = b**2 - 4 * a * c

        if discriminant >= 0:
            sqrt_discriminant = np.sqrt(discriminant)
            t1 = (-b - sqrt_discriminant) / (2 * a)
            t2 = (-b + sqrt_discriminant) / (2 * a)

            if (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1):
                return False

    return True

@njit(cache=True)
def smooth_path(path_nodes, obstacles, clearance):
    """
    パスを平滑化（不要な中間点を削除）
    """
    if len(path_nodes) <= 2:
        return path_nodes

    smoothed_path = [path_nodes[0]]
    i = 0

    while i < len(path_nodes) - 1:
        j = i + 2
        # できるだけ遠くの点まで直接接続できるか確認
        while j < len(path_nodes):
            if not check_edge_validity_simple(
                smoothed_path[-1], path_nodes[j], obstacles, clearance
            ):
                break
            j += 1
        smoothed_path.append(path_nodes[j - 1])
        i = j - 1

    return smoothed_path


def find_path_to_target(
    goal: Position, obstacles: list[Position], clearance=50.0, clearance_factor=0.8,
    calc_total_distance=False
):
    """
    障害物を避けながら目標点への経路を見つける

    :param goal: 目標位置
    :param obstacles: 障害物のリスト
    :param clearance: クリアランス半径
    :param clearance_factor: クリアランスの厳しさを調整する係数。その倍数の大きさの半径でチェックされる
    :return: 経路の座標リスト、総距離
    """
    # func_start = perf_counter()
    start = Position(Position.COORD_LOCAL, 0.0, 0.0)  # 現在位置を原点とするｋ
    start_pos = np.array([start.x, start.y])
    goal_pos = np.array([goal.x, goal.y])

    # list[Position]をndarrayに変換（明示的に2次元配列として作成）
    if len(obstacles) == 0:
        obstacles_array = np.empty((0, 2), dtype=np.float64)
    else:
        obstacles_array = np.array([[obs.x, obs.y] for obs in obstacles], dtype=np.float64)

    # クリアランス円周上のノードを生成
    # start = perf_counter()
    nodes, node_obstacle_map = generate_clearance_circle_nodes(obstacles_array, clearance)
    # end = perf_counter()
    # print("<Pathplan> generate_clearance_circle_nodes() {:.5f}ms".format((end - start) * 1000))

    # start/goalから各障害物への接線ノードを追加
    # start = perf_counter()
    nodes, node_obstacle_map = add_tangent_nodes_for_point(
        start_pos, obstacles_array, clearance, nodes, node_obstacle_map
    )
    nodes, node_obstacle_map = add_tangent_nodes_for_point(
        goal_pos, obstacles_array, clearance, nodes, node_obstacle_map
    )
    # end = perf_counter()
    # print("<Pathplan> add_tangent_nodes_for_point() {:.5f}ms".format((end - start) * 1000))

    print_in_strategy(f"<Pathplan> Generated {len(nodes)} nodes on clearance circles by {len(obstacles)} obstacles.")

    # グラフを構築
    # start = perf_counter()
    adjacency, all_nodes = build_graph(
        nodes, obstacles_array, clearance, clearance_factor, start_pos, goal_pos
    )
    # end = perf_counter()
    # print("<Pathplan> build_graph() {:.5f}ms".format((end - start) * 1000))

    # 最短経路を探索
    # start = perf_counter()
    path_nodes, distance = find_shortest_path(adjacency, all_nodes)
    # end = perf_counter()
    # print("<Pathplan> find_shortest_path() {:.5f}ms".format((end - start) * 1000))


    if len(path_nodes) == 0:
        return [], float("inf")

    # パスを平滑化
    # start = perf_counter()
    smoothed_path_nodes = smooth_path(path_nodes, obstacles_array, clearance)
    # end = perf_counter()
    # print("<Pathplan> smooth_path() {:.5f}ms".format((end - start) * 1000))

    # Position型に変換
    path_coords = [
        Position(Position.COORD_LOCAL, node[0], node[1]) for node in smoothed_path_nodes
    ]

    # 平滑化後の総距離を再計算
    if calc_total_distance:
        total_distance = 0
        for i in range(len(smoothed_path_nodes) - 1):
            total_distance += np.linalg.norm(
                smoothed_path_nodes[i] - smoothed_path_nodes[i + 1]
            )
        print_in_strategy(
            f"<Pathplan> Path found with {len(path_coords)} waypoints, total distance: {total_distance:.2f}"
        )
    else:
        total_distance = distance
        print_in_strategy(
            f"<Pathplan> Path found with {len(path_coords)} waypoints, total distance (before smoothing): {total_distance:.2f}"
        )

    # func_end = perf_counter()
    # print("<Pathplan> <DEBUG> find_path_to_target() {:.5f}ms".format((func_end - func_start) * 1000))
    return path_coords, total_distance


def distance_clearance_factor(goal: Position, obstacles: list[Position], clearance: float) -> float:
    """
    goalがobstaclesのclearanceの範囲に入っているか判定してclearance_factorの値を決める
    敵がボールを持っているか
    """
    clearance_factor = 0.8
    if len(obstacles) == 0:
        return clearance_factor

    for obstacle in obstacles:
        obstacle_pos = np.array([obstacle.x, obstacle.y])
        target_pos = np.array([goal.x, goal.y])
        distance_to_obs = np.linalg.norm(target_pos - obstacle_pos)
        if distance_to_obs < clearance:
            clearance_factor = 0.5
            break
    return clearance_factor


if __name__ == "__main__":
    print("--- Running Path Planning Test ---")

    # テスト用の障害物
    obstacles = [
        Position(Position.COORD_LOCAL, 100.0, 100.0),
        Position(Position.COORD_LOCAL, 300.0, 150.0),
        Position(Position.COORD_LOCAL, 200.0, 300.0),
        Position(Position.COORD_LOCAL, 50.0, 250.0),
        Position(Position.COORD_LOCAL, 350.0, 350.0),
        Position(Position.COORD_LOCAL, 150.0, 350.0),
        Position(Position.COORD_LOCAL, 240.0, 240.0),
        Position(Position.COORD_LOCAL, 350.0, 30.0),
    ]

    goal = Position(Position.COORD_LOCAL, 350.0, 250.0)
    clearance = 50.0
    clearance_factor = 0.8  # クリアランスの厳しさを調整

    # 経路探索
    for i in range(10):
        path_coords, distance = find_path_to_target(
            goal, obstacles, clearance, clearance_factor
        )

    # 結果の表示
    print("\nPath waypoints:")
    for i, waypoint in enumerate(path_coords):
        print(f"  {i}: ({waypoint.x:.2f}, {waypoint.y:.2f})")

    import matplotlib.pyplot as plt

    # 可視化
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # 障害物とクリアランス円
    for obstacle in obstacles:
        # 障害物の中心
        ax.scatter(obstacle.x, obstacle.y, c="red", s=100, marker="o", zorder=5)
        # クリアランス円
        circle = plt.Circle(
            (obstacle.x, obstacle.y),
            clearance,
            fill=False,
            color="red",
            linestyle="--",
            alpha=0.5,
        )
        ax.add_patch(circle)

    # グラフのノードを表示
    start_pos = np.array([0.0, 0.0])
    goal_pos = np.array([goal.x, goal.y])
    if len(obstacles) == 0:
        obstacles_array = np.empty((0, 2), dtype=np.float64)
    else:
        obstacles_array = np.array([[obs.x, obs.y] for obs in obstacles], dtype=np.float64)
    nodes, _ = generate_clearance_circle_nodes(obstacles_array, clearance)
    nodes, _ = add_tangent_nodes_for_point(start_pos, obstacles_array, clearance, nodes, _)
    nodes, _ = add_tangent_nodes_for_point(goal_pos, obstacles_array, clearance, nodes, _)

    if nodes:
        nodes_array = np.array(nodes)
        ax.scatter(
            nodes_array[:, 0],
            nodes_array[:, 1],
            c="blue",
            s=30,
            marker=".",
            alpha=0.5,
            label="Graph nodes",
        )

    # スタートとゴール
    ax.scatter(0, 0, c="green", s=200, marker="s", label="Start", zorder=10)
    ax.scatter(goal.x, goal.y, c="orange", s=200, marker="*", label="Goal", zorder=10)

    # 経路
    if len(path_coords) > 1:
        path_array = np.array([[p.x, p.y] for p in path_coords])
        ax.plot(
            path_array[:, 0],
            path_array[:, 1],
            "g-",
            linewidth=3,
            label=f"Path (dist: {distance:.1f})",
            zorder=8,
        )
        # ウェイポイント
        ax.scatter(
            path_array[1:-1, 0],
            path_array[1:-1, 1],
            c="green",
            s=50,
            marker="o",
            zorder=9,
        )

    ax.set_xlim(-100, 450)
    ax.set_ylim(-100, 450)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Path Planning with Clearance Circle Nodes")

    plt.tight_layout()
    plt.show()
