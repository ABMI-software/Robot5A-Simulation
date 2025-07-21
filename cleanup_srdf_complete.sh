#!/bin/bash
# Script complet pour nettoyer et corriger le fichier SRDF

echo "🧹 Nettoyage complet du fichier SRDF..."

WORKSPACE_PATH="$HOME/Robot5A-Simulation"
SRDF_FILE=$(find $WORKSPACE_PATH -name "armr5.srdf" -type f | head -1)

if [ -z "$SRDF_FILE" ]; then
    echo "❌ Fichier SRDF (armr5.srdf) non trouvé!"
    exit 1
fi

echo "📁 Fichier SRDF trouvé : $SRDF_FILE"

# Sauvegarde
BACKUP_FILE="${SRDF_FILE}.backup_complete_$(date +%Y%m%d_%H%M%S)"
cp "$SRDF_FILE" "$BACKUP_FILE"
echo "💾 Sauvegarde créée : $BACKUP_FILE"

# Compter les problèmes avant nettoyage
ARUCO_BEFORE=$(grep -c "aruco_plane" "$SRDF_FILE" 2>/dev/null || echo "0")
echo "📊 Lignes aruco_plane avant nettoyage : $ARUCO_BEFORE"

# ÉTAPE 1: Supprimer toutes les références aux aruco_plane
echo "🗑️  Suppression de toutes les références aruco_plane..."
sed -i '/aruco_plane/d' "$SRDF_FILE"

# ÉTAPE 2: Ajouter les règles manquantes pour toutes les caméras
echo "📷 Ajout des règles de collision pour toutes les caméras..."

# Créer un fichier temporaire avec les nouvelles règles
TEMP_FILE=$(mktemp)

# Copier tout sauf la balise de fermeture </robot>
sed '$d' "$SRDF_FILE" > "$TEMP_FILE"

# Ajouter les nouvelles règles pour les caméras manquantes
cat >> "$TEMP_FILE" << 'EOF'

    <!-- RÈGLES DE COLLISION CAMÉRAS 3-7 - AJOUTÉES AUTOMATIQUEMENT -->
    
    <!-- Camera 3 collisions -->
    <disable_collisions link1="camera_link3" link2="base_link" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_link1" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_link2" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_link3" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_link4" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_link5" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperLeft_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperLeft_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperLeft_Link3" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperRight_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperRight_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link3" link2="R5A_GripperRight_Link3" reason="Camera"/>

    <!-- Camera 4 collisions -->
    <disable_collisions link1="camera_link4" link2="base_link" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_link1" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_link2" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_link3" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_link4" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_link5" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperLeft_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperLeft_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperLeft_Link3" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperRight_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperRight_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link4" link2="R5A_GripperRight_Link3" reason="Camera"/>

    <!-- Camera 5 collisions (PROBLÉMATIQUE) -->
    <disable_collisions link1="camera_link5" link2="base_link" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_link1" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_link2" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_link3" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_link4" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_link5" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperLeft_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperLeft_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperLeft_Link3" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperRight_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperRight_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link5" link2="R5A_GripperRight_Link3" reason="Camera"/>

    <!-- Camera 6 collisions -->
    <disable_collisions link1="camera_link6" link2="base_link" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_link1" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_link2" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_link3" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_link4" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_link5" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperLeft_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperLeft_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperLeft_Link3" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperRight_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperRight_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link6" link2="R5A_GripperRight_Link3" reason="Camera"/>

    <!-- Camera 7 collisions -->
    <disable_collisions link1="camera_link7" link2="base_link" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_link1" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_link2" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_link3" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_link4" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_link5" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperLeft_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperLeft_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperLeft_Link3" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperRight_Link1" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperRight_Link2" reason="Camera"/>
    <disable_collisions link1="camera_link7" link2="R5A_GripperRight_Link3" reason="Camera"/>

</robot>
EOF

# Remplacer le fichier original
mv "$TEMP_FILE" "$SRDF_FILE"

# Vérifications finales
ARUCO_AFTER=$(grep -c "aruco_plane" "$SRDF_FILE" 2>/dev/null || echo "0")
CAMERAS_TOTAL=$(grep -c "camera_link" "$SRDF_FILE")

echo ""
echo "✅ Nettoyage terminé !"
echo "📊 Références aruco_plane supprimées : $ARUCO_BEFORE"
echo "📊 Références aruco_plane restantes : $ARUCO_AFTER"
echo "📷 Total lignes caméras : $CAMERAS_TOTAL"
echo ""
echo "🔧 Maintenant recompilez :"
echo "   cd $WORKSPACE_PATH"
echo "   colcon build --packages-select robot_moveit_config"
echo "   source install/setup.bash"
echo "   ros2 launch robot_control visual_sim.launch.py"
