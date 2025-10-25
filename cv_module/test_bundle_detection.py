#!/usr/bin/env python3
"""
Test script to validate the optimal bundle detection implementation.
This script tests the bundle configuration and validates the geometry.
"""

import yaml
import numpy as np
import os

def test_bundle_config():
    """Test the bundle configuration file"""
    config_path = "cv_module/config/apriltag3_bundle.yaml"
    
    if not os.path.exists(config_path):
        print(f"❌ Bundle config not found: {config_path}")
        return False
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        bundle_config = config['apriltag3_bundle']['ros__parameters']
        
        print("✅ Bundle config loaded successfully")
        print(f"   Bundle name: {bundle_config['bundle_name']}")
        print(f"   Tag family: {bundle_config['tag_family']}")
        print(f"   Tag size: {bundle_config['tag_size']}m")
        print(f"   Min tags for bundle: {bundle_config['min_tags_for_bundle']}")
        
        # Validate tag corners
        tag_corners = bundle_config['tag_corners']
        print(f"   Tags configured: {len(tag_corners)}")
        
        # Check that all expected tags are present
        expected_tags = [0, 1, 2, 3, 4, 5, 6, 7]
        missing_tags = []
        for tag_id in expected_tags:
            if str(tag_id) not in tag_corners:
                missing_tags.append(tag_id)
        
        if missing_tags:
            print(f"❌ Missing tags: {missing_tags}")
            return False
        
        print("✅ All expected tags present")
        
        # Validate corner positions
        for tag_id_str, corners in tag_corners.items():
            tag_id = int(tag_id_str)
            if len(corners) != 4:
                print(f"❌ Tag {tag_id} has {len(corners)} corners, expected 4")
                return False
            
            # Check that corners form a square
            corners_array = np.array(corners)
            
            # Calculate distances between adjacent corners
            distances = []
            for i in range(4):
                next_i = (i + 1) % 4
                dist = np.linalg.norm(corners_array[i] - corners_array[next_i])
                distances.append(dist)
            
            # All distances should be approximately equal (tag size)
            expected_size = bundle_config['tag_size']
            tolerance = 0.001  # 1mm tolerance
            
            for i, dist in enumerate(distances):
                if abs(dist - expected_size) > tolerance:
                    print(f"❌ Tag {tag_id} corner {i} distance {dist:.4f}m != expected {expected_size:.4f}m")
                    return False
        
        print("✅ All tag corners form valid squares")
        
        # Validate bundle center
        bundle_center = bundle_config['bundle_center']
        if bundle_center != [0.0, 0.0, 0.0]:
            print(f"❌ Bundle center should be [0,0,0], got {bundle_center}")
            return False
        
        print("✅ Bundle center at origin")
        
        return True
        
    except Exception as e:
        print(f"❌ Error loading bundle config: {e}")
        return False

def test_geometry_calculations():
    """Test the geometry calculations"""
    print("\n🔍 Testing geometry calculations...")
    
    # Test tag center calculations
    tag_size = 0.0105  # 10.5mm
    spacing = 0.018    # 18mm center-to-center
    
    # Expected tag centers relative to bundle center
    expected_centers = {
        0: [0.018, -0.018, 0.0],   # Top-left
        1: [0.0, -0.018, 0.0],     # Top-center  
        2: [-0.018, -0.018, 0.0],  # Top-right
        3: [-0.018, 0.0, 0.0],     # Middle-right
        4: [-0.018, 0.018, 0.0],   # Bottom-right
        5: [0.0, 0.018, 0.0],      # Bottom-center
        6: [0.018, 0.018, 0.0],    # Bottom-left
        7: [0.018, 0.0, 0.0],      # Middle-left
    }
    
    # Load actual configuration
    config_path = "cv_module/config/apriltag3_bundle.yaml"
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    bundle_config = config['apriltag3_bundle']['ros__parameters']
    tag_corners = bundle_config['tag_corners']
    
    for tag_id_str, corners in tag_corners.items():
        tag_id = int(tag_id_str)
        corners_array = np.array(corners)
        
        # Calculate center from corners
        center = np.mean(corners_array, axis=0)
        expected_center = np.array(expected_centers[tag_id])
        
        # Check if center matches expected
        error = np.linalg.norm(center - expected_center)
        if error > 0.001:  # 1mm tolerance
            print(f"❌ Tag {tag_id} center error: {error:.4f}m")
            print(f"   Expected: {expected_center}")
            print(f"   Actual: {center}")
            return False
        else:
            print(f"✅ Tag {tag_id} center correct: {center}")
    
    print("✅ All geometry calculations validated")
    return True

def main():
    """Run all tests"""
    print("🧪 Testing Optimal Bundle Detection Implementation")
    print("=" * 60)
    
    tests = [
        ("Bundle Configuration", test_bundle_config),
        ("Geometry Calculations", test_geometry_calculations),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n📋 {test_name}")
        print("-" * 40)
        if test_func():
            passed += 1
            print(f"✅ {test_name} PASSED")
        else:
            print(f"❌ {test_name} FAILED")
    
    print("\n" + "=" * 60)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Bundle detection implementation is ready.")
        print("\n📝 Next steps:")
        print("   1. Build the package: colcon build --packages-select cv_module")
        print("   2. Launch with bundle detection:")
        print("      ros2 launch cv_module full_system_apriltag3.launch.py")
        print("   3. Observe improved accuracy in RViz")
        return True
    else:
        print("❌ Some tests failed. Please fix the issues before proceeding.")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
