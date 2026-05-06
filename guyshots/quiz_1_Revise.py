import argparse
import os
import time
from typing import Optional, Tuple

import numpy as np
import open3d as o3d
import polyscope as ps

from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.preprocessing import StandardScaler
from sklearn.svm import LinearSVC


GROUND_COLOR_REF = np.array([0.6, 0.4, 0.1])
DEFAULT_K = 100
DEFAULT_CV_FOLDS = 10
DEFAULT_MAX_ITER = 2000


# ---------------------------------------------------------------------------
# Data Loading
# ---------------------------------------------------------------------------
def load_ply_point_cloud(path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Load a point cloud from a .ply file.
    Returns points and binary ground truth labels derived from color (1=ground, 0=other).
    """
    pcd = o3d.io.read_point_cloud(path)
    points = np.asarray(pcd.points)
    if points.size == 0:
        raise ValueError(f"No points found in {path}")

    labels = None
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        if colors.shape == points.shape:
            is_ground = np.all(np.isclose(colors, GROUND_COLOR_REF, atol=1e-2), axis=1)
            labels = is_ground.astype(int)

    return points, labels


# ---------------------------------------------------------------------------
# Clustering
# ---------------------------------------------------------------------------
def perform_clustering(points: np.ndarray, k: int) -> np.ndarray:
    """
    Run K-Means clustering on the point cloud.
    """
    print(f"[Step 2] Running K-Means with k={k}...")
    scaler = StandardScaler()
    points_scaled = scaler.fit_transform(points)

    kmeans = KMeans(n_clusters=k, random_state=42, n_init=10)
    cluster_labels = kmeans.fit_predict(points_scaled)

    sizes = np.bincount(cluster_labels, minlength=k)
    print(f"[Step 2] Cluster size summary -> min={sizes.min()}, max={sizes.max()}, mean={sizes.mean():.2f}")
    return cluster_labels


# ---------------------------------------------------------------------------
# PCA Per Cluster
# ---------------------------------------------------------------------------
def safe_unit_vector(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])

    unit = vec / norm
    dominant_idx = int(np.argmax(np.abs(unit)))
    if unit[dominant_idx] < 0:
        unit = -unit
    return unit


def perform_pca(points: np.ndarray, cluster_labels: np.ndarray, k: int):
    """
    Compute PCA separately for each cluster.
    Returns cluster centres, principal component vectors, eigenvalues, and cluster sizes.
    """
    print("[Step 3] Computing PCA for each cluster...")

    cluster_centers = np.zeros((k, 3))
    pca_pc1 = np.zeros((k, 3))
    pca_pc2 = np.zeros((k, 3))
    pca_pc3 = np.zeros((k, 3))
    eigenvalues = np.zeros((k, 3))
    pc3_unit_vectors = np.zeros((k, 3))
    cluster_sizes = np.zeros(k, dtype=int)

    for i in range(k):
        pts = points[cluster_labels == i]
        cluster_sizes[i] = len(pts)

        if len(pts) == 0:
            pca_pc1[i] = np.array([1.0, 0.0, 0.0])
            pca_pc2[i] = np.array([0.0, 1.0, 0.0])
            pca_pc3[i] = np.array([0.0, 0.0, 1.0])
            pc3_unit_vectors[i] = np.array([0.0, 0.0, 1.0])
            print(f"[Step 3] Cluster {i}: empty cluster")
            continue

        cluster_centers[i] = pts.mean(axis=0)

        if len(pts) < 3:
            pca_pc1[i] = np.array([1.0, 0.0, 0.0])
            pca_pc2[i] = np.array([0.0, 1.0, 0.0])
            pca_pc3[i] = np.array([0.0, 0.0, 1.0])
            pc3_unit_vectors[i] = np.array([0.0, 0.0, 1.0])
            print(f"[Step 3] Cluster {i}: {len(pts)} pts | too small for PCA")
            continue

        pca = PCA(n_components=3)
        pca.fit(pts)

        eigenvalues[i] = pca.explained_variance_
        sd = np.sqrt(np.maximum(eigenvalues[i], 0.0))
        pca_pc1[i] = pca.components_[0] * sd[0]
        pca_pc2[i] = pca.components_[1] * sd[1]
        pca_pc3[i] = pca.components_[2] * sd[2]
        pc3_unit_vectors[i] = safe_unit_vector(pca.components_[2])

        print(
            f"[Step 3] Cluster {i}: {len(pts)} pts | "
            f"eigenvalues={eigenvalues[i].round(4)} | pc3_unit={pc3_unit_vectors[i].round(4)}"
        )

    return cluster_centers, pca_pc1, pca_pc2, pca_pc3, eigenvalues, pc3_unit_vectors, cluster_sizes


# ---------------------------------------------------------------------------
# Cluster Ground Truth
# ---------------------------------------------------------------------------
def generate_cluster_ground_truth(cluster_labels: np.ndarray, point_gt_labels: np.ndarray, k: int) -> np.ndarray:
    """
    Assign one label per cluster by majority vote.
    """
    print("[Step 4] Assigning ground truth labels to clusters...")
    cluster_gt = np.zeros(k, dtype=int)

    for i in range(k):
        mask = cluster_labels == i
        total = int(mask.sum())
        if total == 0:
            print(f"[Step 4] Cluster {i}: empty cluster -> label=0")
            continue

        ground_count = int(point_gt_labels[mask].sum())
        cluster_gt[i] = 1 if ground_count > total / 2 else 0
        print(f"[Step 4] Cluster {i}: {ground_count}/{total} ground pts -> label={cluster_gt[i]}")

    return cluster_gt


# ---------------------------------------------------------------------------
# Cluster Feature Matrix
# ---------------------------------------------------------------------------
def build_cluster_feature_matrix(
    cluster_centers: np.ndarray,
    eigenvalues: np.ndarray,
    pc3_unit_vectors: np.ndarray,
    cluster_sizes: np.ndarray,
) -> np.ndarray:
    """
    Build one feature vector per cluster for LinearSVM.
    Each row represents one cluster, not one point.
    Uses eigenvalues, cluster centre, PC3 orientation, and cluster size.
    """
    log_cluster_sizes = np.log1p(cluster_sizes).reshape(-1, 1)
    cluster_feature_matrix = np.hstack([eigenvalues, cluster_centers, pc3_unit_vectors, log_cluster_sizes])
    print(f"[Step 5] Feature matrix shape: {cluster_feature_matrix.shape}")
    print("[Step 5] Features = [eig1, eig2, eig3, centre_x, centre_y, centre_z, pc3_x, pc3_y, pc3_z, log_cluster_size]")
    return cluster_feature_matrix


# ---------------------------------------------------------------------------
# Linear SVM
# ---------------------------------------------------------------------------
def validate_cv_folds(cluster_gt: np.ndarray, requested_folds: int) -> int:
    _, counts = np.unique(cluster_gt, return_counts=True)
    if len(counts) < 2:
        raise ValueError("LinearSVM needs at least 2 cluster classes. Increase k.")

    max_valid_folds = int(np.min(counts))
    if max_valid_folds < 2:
        raise ValueError("Not enough clusters per class for cross-validation. Increase k.")

    return min(requested_folds, max_valid_folds)


def perform_linear_svm(cluster_feature_matrix: np.ndarray, cluster_gt: np.ndarray, cluster_labels: np.ndarray):
    """
    Train and evaluate LinearSVC using cluster-level features and cluster-level labels.
    """
    print("[Step 6] Running LinearSVC on cluster-level features...")

    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(cluster_feature_matrix)
    y_cluster = cluster_gt

    actual_cv_folds = validate_cv_folds(y_cluster, DEFAULT_CV_FOLDS)
    svm = LinearSVC(C=1.0, random_state=42, max_iter=DEFAULT_MAX_ITER)
    cv = StratifiedKFold(n_splits=actual_cv_folds, shuffle=True, random_state=42)

    cv_scores = cross_val_score(svm, X_scaled, y_cluster, cv=cv, scoring="accuracy")
    print(f"[Step 6] Using {actual_cv_folds}-fold CV")
    print(f"[Step 6] CV accuracy: {cv_scores.mean():.3f} +/- {cv_scores.std():.3f}")
    print(f"[Step 6] Scores per fold: {cv_scores.round(3)}")

    svm.fit(X_scaled, y_cluster)

    cluster_preds = svm.predict(X_scaled)
    point_level_predictions = cluster_preds[cluster_labels]
    training_accuracy = float(np.mean(cluster_preds == y_cluster))

    print(f"[Step 6] Cluster training accuracy: {training_accuracy:.3f}")
    print(f"[Step 6] Predicted cluster labels: {cluster_preds}")
    print(f"[Step 6] True cluster labels:      {cluster_gt}")

    return svm, scaler, cluster_preds, point_level_predictions, cv_scores


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
def show_polyscope(
    points: np.ndarray,
    point_gt_labels: np.ndarray,
    cluster_labels: np.ndarray,
    point_preds: np.ndarray,
    cluster_centers: np.ndarray,
    pca_pc1: np.ndarray,
    pca_pc2: np.ndarray,
    pca_pc3: np.ndarray,
    pc3_unit_vectors: np.ndarray,
    cluster_gt: np.ndarray,
    cluster_preds: np.ndarray,
) -> None:
    """
    Visualize the point cloud, clusters, PCA directions, and SVM predictions in Polyscope.
    """
    ps.init()
    ps.set_up_dir("z_up")
    ps.set_ground_plane_mode("none")
    ps.remove_all_structures()

    cloud = ps.register_point_cloud("Processed Point Cloud", points, radius=0.0015)
    cloud.set_point_render_mode("quad")
    cloud.add_scalar_quantity("Elevation", points[:, 2], enabled=True)
    cloud.add_scalar_quantity("Ground Truth", point_gt_labels.astype(float), enabled=False)
    cloud.add_scalar_quantity("Cluster Labels", cluster_labels.astype(float), enabled=False)
    cloud.add_scalar_quantity("LinearSVM Predictions", point_preds.astype(float), enabled=False)

    pca_cloud = ps.register_point_cloud("Cluster Centers", cluster_centers, radius=0.0008)
    pca_cloud.add_vector_quantity("PC1 (Major)", pca_pc1, enabled=True, color=(1, 0, 0), vectortype="ambient")
    pca_cloud.add_vector_quantity("PC2 (Minor)", pca_pc2, enabled=True, color=(0, 1, 0), vectortype="ambient")
    pca_cloud.add_vector_quantity("PC3 (Normal)", pca_pc3, enabled=True, color=(0, 0, 1), vectortype="ambient")
    pca_cloud.add_vector_quantity(
        "PC3 Unit Vector",
        pc3_unit_vectors,
        enabled=False,
        color=(0, 0, 1),
        vectortype="ambient",
    )
    pca_cloud.add_scalar_quantity("Cluster GT", cluster_gt.astype(float), enabled=False)
    pca_cloud.add_scalar_quantity("Cluster Predictions", cluster_preds.astype(float), enabled=False)

    ps.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def get_cache_path(path: str, k: int) -> str:
    stem = os.path.splitext(os.path.basename(path))[0]
    return f"{stem}_k{k}_cluster_labels.npy"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Revised Quiz 1 pipeline: K-Means -> PCA -> cluster-level LinearSVM"
    )
    parser.add_argument("path", nargs="?", default="airport_downsample.ply")
    parser.add_argument("-k", "--clusters", type=int, default=DEFAULT_K, help="Number of clusters")
    parser.add_argument("--cache-clusters", action="store_true", help="Use .npy cache for cluster labels")
    args = parser.parse_args()

    total_t0 = time.perf_counter()

    print("=" * 70)
    print("QUIZ 1 REVISED")
    print("K-Means -> PCA -> Cluster-Level LinearSVM")
    print("=" * 70)
    print(f"[MAIN] Input file: {args.path}")
    print(f"[MAIN] Number of clusters: {args.clusters}")

    # 1. Load Data
    print(f"[Step 1] Loading point cloud from: {args.path}")
    points, point_gt_labels = load_ply_point_cloud(args.path)
    if point_gt_labels is None:
        raise ValueError("Ground truth labels could not be derived from point colours.")

    print(f"[Step 1] Ground points: {int(np.sum(point_gt_labels == 1))}")
    print(f"[Step 1] Non-ground points: {int(np.sum(point_gt_labels == 0))}")
    print(f"[Step 1] Total points loaded: {len(points)}")

    # 2. Clustering
    if args.cache_clusters:
        cache_path = get_cache_path(args.path, args.clusters)
        if os.path.exists(cache_path):
            print(f"[CACHE] Loading cached labels from {cache_path}")
            cluster_labels = np.load(cache_path)
        else:
            cluster_labels = perform_clustering(points, args.clusters)
            np.save(cache_path, cluster_labels)
            print(f"[CACHE] Saved cluster labels to {cache_path}")
    else:
        cluster_labels = perform_clustering(points, args.clusters)

    # 3. Feature Extraction (PCA)
    cluster_centers, pca_pc1, pca_pc2, pca_pc3, eigenvalues, pc3_unit_vectors, cluster_sizes = perform_pca(
        points,
        cluster_labels,
        args.clusters,
    )

    # 4. Ground Truth Generation
    cluster_gt = generate_cluster_ground_truth(cluster_labels, point_gt_labels, args.clusters)

    # 5. Linear SVM Classification
    cluster_feature_matrix = build_cluster_feature_matrix(cluster_centers, eigenvalues, pc3_unit_vectors, cluster_sizes)
    _, _, cluster_preds, point_preds, cv_scores = perform_linear_svm(
        cluster_feature_matrix,
        cluster_gt,
        cluster_labels,
    )

    print("[Summary] Cluster prediction counts:", np.bincount(cluster_preds.astype(int)))
    print("[Summary] Point prediction counts:", np.bincount(point_preds.astype(int)))
    print(f"[Summary] Mean CV accuracy: {cv_scores.mean():.3f}")
    print(f"[TIME] Total pipeline time: {time.perf_counter() - total_t0:.2f} s")

    # 6. Visualization
    show_polyscope(
        points,
        point_gt_labels,
        cluster_labels,
        point_preds,
        cluster_centers,
        pca_pc1,
        pca_pc2,
        pca_pc3,
        pc3_unit_vectors,
        cluster_gt,
        cluster_preds,
    )


if __name__ == "__main__":
    main()
