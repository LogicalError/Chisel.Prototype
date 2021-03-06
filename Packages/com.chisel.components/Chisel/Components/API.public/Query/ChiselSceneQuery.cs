using Chisel.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using Vector2	 = UnityEngine.Vector2;
using Vector3	 = UnityEngine.Vector3;
using Vector4	 = UnityEngine.Vector4;
using GameObject = UnityEngine.GameObject;
using Camera	 = UnityEngine.Camera;
using Plane		 = UnityEngine.Plane;
using System.Runtime.InteropServices;

namespace Chisel.Components
{
    public static class ChiselSceneQuery
    {
#if UNITY_EDITOR
        public static GameObject GetContainerGameObject(GameObject gameObject)
        {
            // TODO: implement finding gameObject that's the container for this gameObject (if applicable, for instance component with "handle as one" flag set)
            //		 might also want to look at HandleUtility.FindSelectionBase(GameObject go) (internal)
            return gameObject;
        }
#endif

        static Dictionary<CSGTreeNode, ChiselIntersection> __foundIntersections = new Dictionary<CSGTreeNode, ChiselIntersection>(); // to avoid allocations
        public static bool FindMultiWorldIntersection(Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, out ChiselIntersection[] intersections)
        {
            return FindMultiWorldIntersection(worldRayStart, worldRayEnd, filterLayerParameter0, visibleLayers, null, null, out intersections);
        }

        static ChiselIntersection Convert(CSGTreeBrushIntersection intersection)
        {
            var node                    = ChiselNodeHierarchyManager.FindChiselNodeByInstanceID(intersection.brush.UserID);
            var model                   = ChiselNodeHierarchyManager.FindChiselNodeByInstanceID(intersection.tree.UserID) as ChiselModel;

            var treeLocalToWorldMatrix  = model.transform.localToWorldMatrix;            
            
            var worldPlaneIntersection  = treeLocalToWorldMatrix.MultiplyPoint(intersection.surfaceIntersection.treePlaneIntersection);
            var worldPlane              = treeLocalToWorldMatrix.TransformPlane(intersection.surfaceIntersection.treePlane);
            
            return new ChiselIntersection()
            {
                node                    = node,
                model                   = model,
                worldPlane              = worldPlane,
                worldPlaneIntersection  = worldPlaneIntersection,
                brushIntersection       = intersection
            };
        }

        public static bool FindMultiWorldIntersection(Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, GameObject[] ignore, GameObject[] filter, out ChiselIntersection[] intersections)
        {
            intersections = null;
            __foundIntersections.Clear();

            HashSet<int> ignoreInstanceIDs = null;
            HashSet<int> filterInstanceIDs = null;
            if (ignore != null)
            {
                ignoreInstanceIDs = new HashSet<int>();
                foreach (var go in ignore)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                        ignoreInstanceIDs.Add(node.GetInstanceID());
                }
            }
            if (filter != null)
            {
                filterInstanceIDs = new HashSet<int>();
                foreach (var go in filter)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                        filterInstanceIDs.Add(node.GetInstanceID());
                }
            }
            
            var allTrees = CSGManager.AllTrees;
            for (var t = 0; t < allTrees.Length; t++)
            {
                var tree	= allTrees[t];
                var model	= ChiselNodeHierarchyManager.FindChiselNodeByTreeNode(tree) as ChiselModel;
                if (!model || !model.isActiveAndEnabled)
                    continue;
                
                if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(model.GetInstanceID())))
                    return false;

                if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(model.GetInstanceID())))
                    return false;

                if (((1 << model.gameObject.layer) & visibleLayers) == 0)
                    continue;

                var query = ChiselMeshQueryManager.GetMeshQuery(model);
                var visibleQueries = ChiselMeshQueryManager.GetVisibleQueries(query);

                // We only accept RayCasts into this model if it's visible
                if (visibleQueries == null ||
                    visibleQueries.Length == 0)
                    return false;

                Vector3 treeRayStart;
                Vector3 treeRayEnd;

                var transform = model.transform;
                if (transform)
                { 
                    var worldToLocalMatrix = transform.worldToLocalMatrix;
                    treeRayStart	= worldToLocalMatrix.MultiplyPoint(worldRayStart);
                    treeRayEnd		= worldToLocalMatrix.MultiplyPoint(worldRayEnd);
                } else
                {
                    treeRayStart	= worldRayStart;
                    treeRayEnd		= worldRayEnd;
                }

                var treeLocalToWorldMatrix = model.transform.localToWorldMatrix;
                var treeIntersections = tree.RayCastMulti(ChiselMeshQueryManager.GetMeshQuery(model), treeRayStart, treeRayEnd, treeLocalToWorldMatrix, filterLayerParameter0);
                if (treeIntersections == null)
                    continue;

                for (var i = 0; i < treeIntersections.Length; i++)
                {
                    var intersection	= treeIntersections[i];
                    var brush			= intersection.brush;
                    var instanceID		= brush.UserID;
                    if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(instanceID)))
                        continue;

                    if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(instanceID)))
                        continue;

                    __foundIntersections[brush] = Convert(intersection);
                }
            }

            if (__foundIntersections.Count == 0)
                return false;

            var sortedIntersections = __foundIntersections.Values.ToArray();
            Array.Sort(sortedIntersections, (x, y) => (x.brushIntersection.surfaceIntersection.distance < y.brushIntersection.surfaceIntersection.distance) ? -1 : 0);

            __foundIntersections.Clear();
            intersections = sortedIntersections;
            return true;
        }

        public static bool FindFirstWorldIntersection(Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, out ChiselIntersection foundIntersection)
        {
            return FindFirstWorldIntersection(worldRayStart, worldRayEnd, filterLayerParameter0, visibleLayers, null, null, out foundIntersection);
        }

        public static bool FindFirstWorldIntersection(Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, GameObject[] ignore, GameObject[] filter, out ChiselIntersection foundIntersection)
        {
            bool found = false;
            foundIntersection = ChiselIntersection.None;

            HashSet<int> ignoreInstanceIDs = null;
            HashSet<int> filterInstanceIDs = null;
            if (ignore != null)
            {
                ignoreInstanceIDs = new HashSet<int>();
                foreach (var go in ignore)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                        ignoreInstanceIDs.Add(node.GetInstanceID());
                }
            }
            if (filter != null)
            {
                filterInstanceIDs = new HashSet<int>();
                foreach (var go in filter)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                        filterInstanceIDs.Add(node.GetInstanceID());
                }
            }

            var allTrees = CSGManager.AllTrees;
            for (var t = 0; t < allTrees.Length; t++)
            {
                var tree	= allTrees[t];
                var model	= ChiselNodeHierarchyManager.FindChiselNodeByTreeNode(tree) as ChiselModel;
                if (!model || !model.isActiveAndEnabled)
                    continue;
                
                if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(model.GetInstanceID())))
                    return false;

                if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(model.GetInstanceID())))
                    return false;

                if (((1 << model.gameObject.layer) & visibleLayers) == 0)
                    continue;

                var query = ChiselMeshQueryManager.GetMeshQuery(model);
                var visibleQueries = ChiselMeshQueryManager.GetVisibleQueries(query);

                // We only accept RayCasts into this model if it's visible
                if (visibleQueries == null ||
                    visibleQueries.Length == 0)
                    return false;

                Vector3 treeRayStart;
                Vector3 treeRayEnd;

                var transform = model.transform;
                if (transform)
                { 
                    var worldToLocalMatrix = transform.worldToLocalMatrix;
                    treeRayStart	= worldToLocalMatrix.MultiplyPoint(worldRayStart);
                    treeRayEnd		= worldToLocalMatrix.MultiplyPoint(worldRayEnd);
                } else
                {
                    treeRayStart	= worldRayStart;
                    treeRayEnd		= worldRayEnd;
                }

                var treeLocalToWorldMatrix = model.transform.localToWorldMatrix;
                var treeIntersections = tree.RayCastMulti(ChiselMeshQueryManager.GetMeshQuery(model), treeRayStart, treeRayEnd, treeLocalToWorldMatrix, filterLayerParameter0);
                if (treeIntersections == null)
                    continue;

                for (var i = 0; i < treeIntersections.Length; i++)
                {
                    var intersection	= treeIntersections[i];
                    var brush			= intersection.brush;
                    var instanceID		= brush.UserID;

                    if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(instanceID)))
                        continue;

                    if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(instanceID)))
                        continue;

                    if (intersection.surfaceIntersection.distance < foundIntersection.brushIntersection.surfaceIntersection.distance)
                    {
                        foundIntersection = Convert(intersection);
                        found = true;
                    }
                }
            }
            return found;
        }
        
        
        public static bool FindFirstWorldIntersection(ChiselModel model, Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, out ChiselIntersection foundIntersection)
        {
            return FindFirstWorldIntersection(model, worldRayStart, worldRayEnd, filterLayerParameter0, visibleLayers, null, null, out foundIntersection);
        }

        public static bool FindFirstWorldIntersection(ChiselModel model, Vector3 worldRayStart, Vector3 worldRayEnd, int filterLayerParameter0, int visibleLayers, GameObject[] ignore, GameObject[] filter, out ChiselIntersection foundIntersection)
        {
            foundIntersection = ChiselIntersection.None;

            if (!model || !model.isActiveAndEnabled)
                return false;

            CSGTreeNode[] ignoreBrushes = null;
            HashSet<int> ignoreInstanceIDs = null;
            HashSet<int> filterInstanceIDs = null;
            if (ignore != null)
            {
                //var ignoreBrushList = new HashSet<CSGTreeBrush>();
                ignoreInstanceIDs = new HashSet<int>();
                foreach (var go in ignore)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                    {
                        //node.GetAllTreeBrushes(ignoreBrushList);
                        ignoreInstanceIDs.Add(node.GetInstanceID());
                    }
                }/*
                if (ignoreBrushList.Count > 0)
                {
                    // TODO: fix this, ignorebrushes doesn't remove the brush completely, but that's not necessarily correct
                    //			for example: another brush that is not ignored, subtracts from a brush that's ignored (so its insides should be selectable)
                    ignoreBrushes = new CSGTreeNode[ignoreBrushList.Count];
                    int index = 0;
                    foreach(var brush in ignoreBrushList)
                    {
                        ignoreBrushes[index] = brush;
                        index++;
                    }
                }*/
            }
            if (filter != null)
            {
                filterInstanceIDs = new HashSet<int>();
                foreach (var go in filter)
                {
                    var node = go.GetComponent<ChiselNode>();
                    if (node)
                        filterInstanceIDs.Add(node.GetInstanceID());
                }
            }
            

            var tree	= model.Node;
            if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(model.GetInstanceID())))
                return false;

            if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(model.GetInstanceID())))
                return false;

            if (((1 << model.gameObject.layer) & visibleLayers) == 0)
                return false;
            
            var query = ChiselMeshQueryManager.GetMeshQuery(model);
            var visibleQueries = ChiselMeshQueryManager.GetVisibleQueries(query);

            // We only accept RayCasts into this model if it's visible
            if (visibleQueries == null ||
                visibleQueries.Length == 0)
                return false;

            Vector3 treeRayStart;
            Vector3 treeRayEnd;

            var transform = model.transform;
            if (transform)
            { 
                var worldToLocalMatrix = transform.worldToLocalMatrix;
                treeRayStart	= worldToLocalMatrix.MultiplyPoint(worldRayStart);
                treeRayEnd		= worldToLocalMatrix.MultiplyPoint(worldRayEnd);
            } else
            {
                treeRayStart	= worldRayStart;
                treeRayEnd		= worldRayEnd;
            }

            var treeLocalToWorldMatrix = model.transform.localToWorldMatrix;
            var treeIntersections = tree.RayCastMulti(ChiselMeshQueryManager.GetMeshQuery(model), treeRayStart, treeRayEnd, treeLocalToWorldMatrix, filterLayerParameter0, ignoreBrushes);
            if (treeIntersections == null)
                return false;
            
            bool found = false;
            for (var i = 0; i < treeIntersections.Length; i++)
            {
                var intersection	= treeIntersections[i];
                var brush			= intersection.brush;
                var instanceID		= brush.UserID;
                
                if ((filterInstanceIDs != null && !filterInstanceIDs.Contains(instanceID)))
                    continue;

                if ((ignoreInstanceIDs != null && ignoreInstanceIDs.Contains(instanceID)))
                    continue;

                if (intersection.surfaceIntersection.distance < foundIntersection.brushIntersection.surfaceIntersection.distance)
                {
                    foundIntersection = Convert(intersection);
                    found = true;
                }
            }
            return found;
        }
        
        public static bool GetNodesInFrustum(Frustum frustum, int visibleLayers, ref HashSet<CSGTreeNode> rectFoundNodes)
        {
            rectFoundNodes.Clear();
            var planes			= new Plane[6];
            Vector4 srcVector;
            var allTrees		= CSGManager.AllTrees;
            for (var t = 0; t < allTrees.Length; t++)
            {
                var tree	= allTrees[t];
                var model	= ChiselNodeHierarchyManager.FindChiselNodeByTreeNode(tree) as ChiselModel;
                if (!model || !model.isActiveAndEnabled)
                    continue;

                if (((1 << model.gameObject.layer) & visibleLayers) == 0)
                    continue;

                var query = ChiselMeshQueryManager.GetMeshQuery(model);
                var visibleQueries = ChiselMeshQueryManager.GetVisibleQueries(query);

                // We only accept RayCasts into this model if it's visible
                if (visibleQueries == null ||
                    visibleQueries.Length == 0)
                    continue;
                
                // Transform the frustum into the space of the tree				
                var transform								= model.transform;
                var worldToLocalMatrixInversed				= transform.localToWorldMatrix;	 // localToWorldMatrix == worldToLocalMatrix.inverse
                var worldToLocalMatrixInversedTransposed	= worldToLocalMatrixInversed.transpose;
                for (int p = 0; p < 6; p++)
                {
                    var srcPlane  = frustum.Planes[p];
                    srcVector.x = srcPlane.normal.x;
                    srcVector.y = srcPlane.normal.y;
                    srcVector.z = srcPlane.normal.z;
                    srcVector.w = srcPlane.distance;

                    srcVector = worldToLocalMatrixInversedTransposed * srcVector;

                    planes[p].normal   = srcVector;
                    planes[p].distance = srcVector.w;
                }

                var treeNodesInFrustum = tree.GetNodesInFrustum(query, planes);
                if (treeNodesInFrustum == null)
                    continue;

                for (int n = 0; n < treeNodesInFrustum.Length; n++)
                {
                    var treeNode		= treeNodesInFrustum[n];
                    rectFoundNodes.Add(treeNode);
                }
            }
            return rectFoundNodes.Count > 0;
        }
    }
}
