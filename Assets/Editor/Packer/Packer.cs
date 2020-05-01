using Chisel.Components;
using NUnit.Framework.Constraints;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq.Expressions;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnitySceneExtensions;
using Grid = UnitySceneExtensions.Grid;
using Debug = UnityEngine.Debug;
using System.Linq;
using Unity.Mathematics;
using System.Numerics;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;
using Vector4 = UnityEngine.Vector4;

namespace Chisel.Editors
{
    /// Mesh topology information.
    public class MeshTopology
    {
        public MeshTopology(ChartMesh mesh) { buildTopologyInfo(mesh); }

        /// Determine if the mesh is connected.
        public bool isConnected()  { return m_connectedCount == 1; }

        /// Determine if the mesh is closed. (Each edge is shared by two faces)
        public bool isClosed()  { return m_boundaryCount == 0; }

        /// Return true if the mesh has the topology of a disk.
        public bool isDisk()  { return isConnected() && m_boundaryCount == 1/* && m_eulerNumber == 1*/; }

        /// Return the number of connected components.
        public int connectedCount()  { return m_connectedCount; }

        /// Return the number of open holes.
        public int holeCount()  { return m_boundaryCount; }

        /// Return the genus of the mesh.
        public int genus()  { return m_genus; }

        /// Return the euler number of the mesh.
        public int euler()  { return m_eulerNumber; }


        public static bool isQuadOnly(ChartMesh mesh)
        {
            var faceCount = mesh.faceLength;
            for (var f = 0; f < faceCount; f++)
            {
                var face = mesh.faceAt(f);
                if (face.edgeLength != 4)
                {
                    return false;
                }
            }

            return true;
        }


        const uint NIL = unchecked((uint)(~0u));

        void buildTopologyInfo(ChartMesh mesh)
        {
            var vertexCount = mesh.colocalVertexCount;
            var faceCount = mesh.faceLength;
            var edgeCount = mesh.edgeLength;

            //Debug.Log( "--- Building mesh topology:\n" );

            var stack = new Stack(faceCount);

            BitArray bitFlags = new BitArray((uint)faceCount);
            bitFlags.clearAll();

            // Compute connectivity.
            //Debug.Log( "---   Computing connectivity.\n" );

            m_connectedCount = 0;

            for(var f = 0; f < faceCount; f++ ) 
            {
                if( bitFlags.bitAt((uint)f) == false ) 
                {
                    m_connectedCount++;

                    stack.Push( f );
                    while( stack.Count > 0 ) {

                        var topobj = stack.Pop();
                        Debug.Assert(topobj != null);
                        uint top = (uint)topobj;
                        Debug.Assert(top != NIL);

                        if ( bitFlags.bitAt(top) == false ) {
                            bitFlags.setBitAt(top);

                            var face = mesh.faceAt((int)top);
                            var firstEdge = face.edge;
                            var edge = firstEdge;

                            do {
                                var neighborFace = edge.pair.face;
                                if (neighborFace != null) {
                                    stack.Push(neighborFace.id);
                                }
                                edge = edge.next;
                            } while(edge != firstEdge);
                        }
                    }
                }
            }
            Debug.Assert(stack.Count == 0);
            //Debug.Log( "---   %d connected components.\n", m_connectedCount );


            // Count boundary loops.
            //Debug.Log( "---   Counting boundary loops.\n" );
            m_boundaryCount = 0;

            bitFlags.resize((uint)edgeCount);
            bitFlags.clearAll();

            // Don't forget to link the boundary otherwise this won't work.
            for (var e = 0; e < edgeCount; e++)
            {
                var startEdge = mesh.edgeAt(e);
                if (startEdge != null && startEdge.isBoundary() && bitFlags.bitAt((uint)e) == false)
                {
                    Debug.Assert(startEdge.face != null);
                    Debug.Assert(startEdge.pair.face == null);

                    startEdge = startEdge.pair;

                    m_boundaryCount++;

                    var edge = startEdge;
                    do {
                        bitFlags.setBitAt(edge.id / 2);
                        edge = edge.next;
                    } while(startEdge != edge);
                }
            }
            //Debug.Log("---   %d boundary loops found.\n", m_boundaryCount );


            // Compute euler number.
            m_eulerNumber = vertexCount - edgeCount + faceCount;
            //Debug.Log("---   Euler number: %d.\n", m_eulerNumber);


            // Compute genus. (only valid on closed connected surfaces)
            m_genus = -1;
            if( isClosed() && isConnected() ) {
                m_genus = (2 - m_eulerNumber) / 2;
                //Debug.Log("---   Genus: %d.\n", m_genus);
            }
        }

        ///< Number of boundary loops.
        int m_boundaryCount;		

        ///< Number of connected components.
        int m_connectedCount;		

        ///< Euler number.
        int m_eulerNumber;

        /// Mesh genus.
        int m_genus;
    };


    class Chart
    {
        public Chart()
        {
            m_chartMesh = null; m_unifiedMesh = null; m_isDisk = false; m_isVertexMapped = false;
        }
        
        ChartMesh unifyVertices(ChartMesh inputMesh, List<uint> chartToUnifiedMap, uint chartToUnifiedMapCount)
        {
            var mesh = new ChartMesh();

            // Only add the first colocal.
            var inputMesh_to_newMesh = new uint[inputMesh.vertexLength];
            for (int i = 0; i < inputMesh_to_newMesh.Length; i++)
                inputMesh_to_newMesh[i] = unchecked((uint)~0u);

            for (var v = 0; v < inputMesh.vertexLength; v++) {
                var vertex = inputMesh.vertexAt(v);
                if (vertex.isFirstColocal()) {
                    var vtx = mesh.addVertex(vertex.pos);
                    vtx.nor = vertex.nor;
                    vtx.tex = vertex.tex;
                    vtx.col = vertex.col;
                    inputMesh_to_newMesh[v] = vtx.id;
                }
            }
            // patch chartToUnifiedMap
            for (var v = 0; v < inputMesh.vertexLength; v++) {
                var vertex = inputMesh.vertexAt(v);
                uint colocal_id = vertex.isFirstColocal() ? vertex.id : vertex.firstColocal().id;
                uint new_vertex_idx = inputMesh_to_newMesh[colocal_id];
                Debug.Assert(new_vertex_idx != unchecked ((uint)~0u));
                Debug.Assert(new_vertex_idx < mesh.vertexLength);
                for (var i = 0; i < chartToUnifiedMapCount; i++) {
                    if (chartToUnifiedMap[i] == vertex.id) {
                        chartToUnifiedMap[i] = new_vertex_idx;
                    }
                }
            }

            var indexArray = new List<uint>();

            // Add new faces pointing to first colocals.
            var faceCount = inputMesh.faceLength;
            for (var f = 0; f < faceCount; f++) {
                var face = inputMesh.faceAt(f);

                indexArray.Clear();

                for (var it = face.edges(); !it.isDone(); it.advance()) {
                    var edge = it.current();
                    var vertex = edge.vertex.firstColocal();
                    uint vtx_idx = inputMesh_to_newMesh[vertex.id];
                    Debug.Assert(vtx_idx != unchecked ((uint)-1));
                    //indexArray.Add(vertex.id);
                    indexArray.Add(vtx_idx);
                }

                mesh.addFace(indexArray);
            }

            mesh.linkBoundary();

            return mesh;
        }



        static void getBoundaryEdges(ChartMesh mesh, List<Edge> boundaryEdges)
        {
            Debug.Assert(mesh != null);

            var edgeCount = mesh.edgeLength;

            var bitFlags = new BitArray((uint)edgeCount);
            bitFlags.clearAll();

            boundaryEdges.Clear();

            // Search for boundary edges. Mark all the edges that belong to the same boundary.
            for (var e = 0; e < edgeCount; e++)
            {
                var startEdge = mesh.edgeAt(e);

                if (startEdge != null && startEdge.isBoundary() && bitFlags.bitAt((uint)e) == false)
                {
                    Debug.Assert(startEdge.face != null);
                    Debug.Assert(startEdge.pair.face == null);

                    startEdge = startEdge.pair;

                    var edge = startEdge;
                    do {
                        Debug.Assert(edge.face == null);
                        Debug.Assert(bitFlags.bitAt(edge.id/2) == false);

                        bitFlags.setBitAt(edge.id / 2);
                        edge = edge.next;
                    } while(startEdge != edge);

                    boundaryEdges.Add(startEdge);
                }
            }
        }
        
        bool closeLoop(int start, List<Edge> loop)
        {
            var vertexCount = loop.Count - start;

            Debug.Assert(vertexCount >= 3);
            if (vertexCount < 3) return false;

            Debug.Assert(loop[start].vertex.isColocal(loop[start+vertexCount-1].to()));

            // If the hole is planar, then we add a single face that will be properly triangulated later.
            // If the hole is not planar, we add a triangle fan with a vertex at the hole centroid.
            // This is still a bit of a hack. There surely are better hole filling algorithms out there.

            var points = new Vector3[vertexCount];
            for (var i = 0; i < vertexCount; i++) {
                points[i] = loop[start + i].vertex.pos;
            }

            bool isPlanar = MeshCharts.isPlanar(vertexCount, points);

            if (isPlanar) {
                // Add face and connect edges.
                var face = m_unifiedMesh.addFace();
                for (var i = 0; i < vertexCount; i++) 
                {
                    var edge = loop[start + i];
            
                    edge.face = face;
                    edge.setNext(loop[start + (i + 1) % vertexCount]);
                }
                face.edge = loop[start];

                Debug.Assert(face.isValid());
            }
            else {
                // If the polygon is not planar, we just cross our fingers, and hope this will work:

                // Compute boundary centroid:
                var centroidPos = Vector3.zero;
                for (uint i = 0; i < vertexCount; i++) {
                    centroidPos += points[i];
                }
                centroidPos *= (1.0f / vertexCount);

                var centroid = m_unifiedMesh.addVertex(centroidPos);

                // Add one pair of edges for each boundary vertex.
                for (int j = vertexCount-1, i = 0; i < vertexCount; j = i++) {
                    var face = m_unifiedMesh.addFace(centroid.id, loop[start+j].vertex.id, loop[start+i].vertex.id);
                    Debug.Assert(face != null);
                }
            }

            return true;
        }
        
        bool closeHoles()
        {
            Debug.Assert(!m_isVertexMapped);

            var boundaryEdges = new List<Edge>();
            getBoundaryEdges(m_unifiedMesh, boundaryEdges);

            var boundaryCount = boundaryEdges.Count;
            if (boundaryCount <= 1)
            {
                // Nothing to close.
                return true;
            }

            // Compute lengths and areas.
            var boundaryLengths = new List<float>();

            for (var i = 0; i < boundaryCount; i++)
            {
                var startEdge = boundaryEdges[i];
                Debug.Assert(startEdge.face == null);

                float boundaryLength = 0.0f;

                var edge = startEdge;
                do {
                    var t0 = edge.from().pos;
                    var t1 = edge.to().pos;

                    //boundaryEdgeCount++;
                    boundaryLength += (t1 - t0).magnitude;
                    //boundaryCentroid += edge.vertex().pos;

                    edge = edge.next;
                } while(edge != startEdge);

                boundaryLengths.Add(boundaryLength);
                //boundaryCentroids.Add(boundaryCentroid / boundaryEdgeCount);
            }


            // Find disk boundary.
            uint diskBoundary = 0;
            float maxLength = boundaryLengths[0];

            for (var i = 1; i < boundaryCount; i++)
            {
                if (boundaryLengths[i] > maxLength)
                {
                    maxLength = boundaryLengths[i];
                    diskBoundary = (uint)i;
                }
            }


            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // @@ Close loop is wrong, after closing a loop, we do not only have to add the face, but make sure that every edge in he loop is pointing to the right place.

            // Close holes.
            for (var i = 0; i < boundaryCount; i++)
            {
                if (diskBoundary == i)
                {
                    // Skip disk boundary.
                    continue;
                }

                var startEdge = boundaryEdges[i];
                Debug.Assert(startEdge != null);
                Debug.Assert(startEdge.face == null);

                var vertexLoop = new List<Vertex>();
                var edgeLoop = new List<Edge>();

                var edge = startEdge;
                do {
                    var vertex = edge.next.vertex; // edge.to()

                    int n;
                    for (n = 0; n < vertexLoop.Count; n++) {
                        if (vertex.isColocal(vertexLoop[n])) {
                            break;
                        }
                    }
            
                    bool isCrossing = (n != vertexLoop.Count);

                    if (isCrossing) {

                        var prev = edgeLoop[n];    // Previous edge before the loop.
                        var next = edge.next;   // Next edge after the loop.

                        Debug.Assert(prev.to().isColocal(next.from()));

                        // Close loop.
                        edgeLoop.Add(edge);
                        closeLoop(n+1, edgeLoop);

                        // Link boundary loop.
                        prev.setNext(next);
                        vertex.setEdge(next);

                        // Start over again.
                        vertexLoop.Clear();
                        edgeLoop.Clear();
                
                        edge = startEdge;
                        vertex = edge.to();
                    }

                    vertexLoop.Add(vertex);
                    edgeLoop.Add(edge);

                    edge = edge.next;
                } while(edge != startEdge);

                closeLoop(0, edgeLoop);
            }

            getBoundaryEdges(m_unifiedMesh, boundaryEdges);

            boundaryCount = boundaryEdges.Count;
            Debug.Assert(boundaryCount == 1);

            //exportMesh(m_unifiedMesh.ptr(), "debug_hole_filled.obj");

            return boundaryCount == 1;
        }

        bool isTriangularMesh(ChartMesh mesh)
        {
            for (var  it = mesh.faces(); !it.isDone(); it.advance())
            {
                var face = it.current();
                if (face.edgeLength != 3) 
                    return false;
            }
            return true;
        }
        
        static bool pointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            return  Face.triangleArea(a, b, p) >= 0.00001f &&
                    Face.triangleArea(b, c, p) >= 0.00001f &&
                    Face.triangleArea(c, a, p) >= 0.00001f; 
        }

        
        // This is doing a simple ear-clipping algorithm that skips invalid triangles. Ideally, we should
        // also sort the ears by angle, start with the ones that have the smallest angle and proceed in order.
        ChartMesh triangulate(ChartMesh inputMesh)
        {
            var mesh = new ChartMesh();
    
            // Add all vertices.
            var vertexCount = inputMesh.vertexLength;
            for (var v = 0; v < vertexCount; v++) 
            {
                var vertex = inputMesh.vertexAt(v);
                var new_vertex = mesh.addVertex(vertex.pos);
                new_vertex.nor = vertex.nor;
                new_vertex.tex = vertex.tex;
                new_vertex.col = vertex.col;
            }

            mesh.colocalVertexCount = inputMesh.colocalVertexCount;

            var polygonVertices = new List<int>();
            var polygonPoints = new List<Vector2>();

            var faceCount = inputMesh.faceLength;
            for (var f = 0; f < faceCount; f++)
            {
                var face = inputMesh.faceAt(f);
                Debug.Assert(face != null);

                var edgeCount = face.edgeLength;
                Debug.Assert(edgeCount >= 3);

                polygonVertices.Clear();
                polygonVertices.Capacity = edgeCount;

                if (edgeCount == 3) {
                    // Simple case for triangles.
                    for (var it = face.edges(); !it.isDone(); it.advance())
                    {
                        var edge = it.current();
                        var vertex = edge.vertex;
                        polygonVertices.Add((int)vertex.id);
                    }

                    int v0 = polygonVertices[0];
                    int v1 = polygonVertices[1];
                    int v2 = polygonVertices[2];

                    mesh.addFace((uint)v0, (uint)v1, (uint)v2);
                }
                else {
                    // Build 2D polygon projecting vertices onto normal plane.
                    // Faces are not necesarily planar, this is for example the case, when the face comes from filling a hole. In such cases
                    // it's much better to use the best fit plane.
                    var fn = face.normal();

                    Basis basis = new Basis();
                    basis.buildFrameForDirection(fn);

                    polygonPoints.Clear();
                    polygonPoints.Capacity = edgeCount;

                    for (var it = face.edges(); !it.isDone(); it.advance())
                    {
                        var edge = it.current();
                        var vertex = edge.vertex;
                        polygonVertices.Add((int)vertex.id);
                
                        Vector2 p;
                        p.x = Vector3.Dot(basis.tangent, vertex.pos);
                        p.y = Vector3.Dot(basis.bitangent, vertex.pos);

                        polygonPoints.Add(p);
                    }

                    var polygonAngles = new List<float>(edgeCount);
                    for (int i = 0; i < edgeCount; i++)
                        polygonAngles.Add(0);

                    while (polygonVertices.Count > 2) {
                        var size = polygonVertices.Count;

                        // Update polygon angles. @@ Update only those that have changed.
                        float minAngle = 2 * Mathf.PI;
                        uint bestEar = 0; // Use first one if none of them is valid.
                        bool bestIsValid = false;
                        for (uint i = 0; i < size; i++) 
                        {
                            var i0 = i;
                            var i1 = (i+1) % size; // Use Sean's polygon interation trick.
                            var i2 = (i+2) % size;

                            Vector2 p0 = polygonPoints[(int)i0];
                            Vector2 p1 = polygonPoints[(int)i1];
                            Vector2 p2 = polygonPoints[(int)i2];

                            float d = Mathf.Clamp(Vector3.Dot(p0-p1, p2-p1) / ((p0-p1).magnitude * (p2-p1).magnitude), -1.0f, 1.0f);
                            float angle = Mathf.Acos(d);
                    
                            float area = Face.triangleArea(p0, p1, p2);
                            if (area < 0.0f) angle = 2.0f * Mathf.PI - angle;

                            polygonAngles[(int)i1] = angle;

                            if (angle < minAngle || !bestIsValid) {

                                // Make sure this is a valid ear, if not, skip this point.
                                bool valid = true;
                                for (uint j = 0; j < size; j++) {
                                    if (j == i0 || j == i1 || j == i2) continue;
                                    Vector2 p = polygonPoints[(int)j];

                                    if (pointInTriangle(p, p0, p1, p2)) {
                                        valid = false;
                                        break;
                                    }
                                }

                                if (valid || !bestIsValid) {
                                    minAngle = angle;
                                    bestEar = (uint)i1;
                                    bestIsValid = valid;
                                }
                            }
                        }

                        Debug.Assert(minAngle <= 2 * Mathf.PI);

                        // Clip best ear:

                        {
                            var i0 = (bestEar + size - 1) % size;
                            var i1 = (bestEar + 0) % size;
                            var i2 = (bestEar + 1) % size;

                            int v0 = polygonVertices[(int)i0];
                            int v1 = polygonVertices[(int)i1];
                            int v2 = polygonVertices[(int)i2];

                            mesh.addFace((uint)v0, (uint)v1, (uint)v2);

                            polygonVertices.RemoveAt((int)i1);
                            polygonPoints.RemoveAt((int)i1);
                            polygonAngles.RemoveAt((int)i1);
                        }
                    }
                }

            }

            mesh.linkBoundary();

            return mesh;
        }

        public void build(ChartMesh originalMesh, List<uint> faceArray, bool from_uvs = false)
        {
            // Copy face indices.
            m_faceArray = faceArray.ToList();

            var meshVertexCount = originalMesh.vertexLength;

            m_chartMesh = new ChartMesh();
            m_unifiedMesh = new ChartMesh();

            uint[] chartMeshIndices = new uint[meshVertexCount];
            for (int i = 0; i < meshVertexCount; i++)
                chartMeshIndices[i] = unchecked ((uint)~0u);

            uint[] unifiedMeshIndices = new uint[meshVertexCount];
            for (int i = 0; i < meshVertexCount; i++)
                unifiedMeshIndices[i] = unchecked((uint)~0u);

            var uvVertexMap = new Dictionary<Vector2, uint>();

            // Add vertices.
            var faceCount = faceArray.Count;
            for (var f = 0; f < faceCount; f++)
            {
                var face = originalMesh.faceAt((int)faceArray[f]);
                Debug.Assert(face != null);

                for(var it = face.edges(); !it.isDone(); it.advance())
                {
                    var vertex = it.current().vertex;
                    var unifiedVertex = vertex.firstColocal();

                    if (from_uvs) {
                        // @@ Unify vertices on the fly, instead of relying on the colocals?
                        
                        if (!uvVertexMap.TryGetValue(vertex.tex, out uint index)) {
                            var v = m_unifiedMesh.addVertex(new Vector3(vertex.tex.x, vertex.tex.y, 0));
                            v.nor = vertex.pos; // This is a hack to easily restore the positions later! We can probably do this through the index buffers, but this is more simple.
                            v.tex = vertex.tex;
                        } else
                            index = (uint)m_unifiedMesh.vertexLength;

                        // This maps original . unified vertex indices.
                        if (unifiedMeshIndices[vertex.id] == unchecked (~0u)) {
                            unifiedMeshIndices[vertex.id] = index;
                        }

                        // This maps original . chart vertex indices.
                        if (chartMeshIndices[vertex.id] == unchecked(~0u)) {
                            chartMeshIndices[vertex.id] = index;
                        }

                        if (index != m_unifiedMesh.vertexLength) {
                            var v = m_chartMesh.addVertex(new Vector3(vertex.tex.x, vertex.tex.y, 0));
                            v.nor = vertex.pos; // This is a hack to easily restore the positions later! We can probably do this through the index buffers, but this is more simple.
                            v.tex = vertex.tex;

                            m_chartToOriginalMap.Add(vertex.id);
                            m_chartToUnifiedMap.Add(unifiedMeshIndices[vertex.id]); // index?
                        }
                    } else 
                    {
                        if (unifiedMeshIndices[unifiedVertex.id] == unchecked(~0u))
                        {
                            unifiedMeshIndices[unifiedVertex.id] = (uint)m_unifiedMesh.vertexLength;

                            Debug.Assert(vertex.pos == unifiedVertex.pos);
                            var v = m_unifiedMesh.addVertex(vertex.pos);
                            v.tex = vertex.tex;
                        }

                        if (chartMeshIndices[vertex.id] == unchecked(~0u))
                        {
                            chartMeshIndices[vertex.id] = (uint)m_chartMesh.vertexLength;
                            m_chartToOriginalMap.Add(vertex.id);
                            m_chartToUnifiedMap.Add(unifiedMeshIndices[unifiedVertex.id]);

                            var v = m_chartMesh.addVertex(vertex.pos);
                            v.nor = vertex.nor;
                            v.tex = vertex.tex;
                        }
                    }
                }
            }

            // This is ignoring the canonical map:
            // - Is it really necessary to link colocals?

            m_chartMesh.linkColocals();    
            //m_unifiedMesh.linkColocals();  // Not strictly necessary, no colocals in the unified mesh. # Wrong.
            m_unifiedMesh.colocalVertexCount = m_unifiedMesh.vertexLength;

            if (from_uvs) {
                m_unifiedMesh.linkColocals();
            }

            // This check is not valid anymore, if the original mesh vertices were linked with a canonical map, then it might have
            // some colocal vertices that were unlinked. So, the unified mesh might have some duplicate vertices, because firstColocal()
            // is not guaranteed to return the same vertex for two colocal vertices.
            //Debug.Assert(m_chartMesh.colocalVertexLength == m_unifiedMesh.vertexLength);

            // Is that OK? What happens in meshes were that happens? Does anything break? Apparently not...


            var faceIndices = new List<uint>(7);

            // Add faces.
            for (var f = 0; f < faceCount; f++)
            {
                var face = originalMesh.faceAt((int)faceArray[f]);
                Debug.Assert(face != null);

                faceIndices.Clear();

                for(var it = face.edges(); !it.isDone(); it.advance())
                {
                    var vertex = it.current().vertex;
                    Debug.Assert(vertex != null);

                    faceIndices.Add(chartMeshIndices[vertex.id]);
                }

                m_chartMesh.addFace(faceIndices);

                faceIndices.Clear();

                for(var it = face.edges(); !it.isDone(); it.advance())
                {
                    var vertex = it.current().vertex;
                    Debug.Assert(vertex != null);

                    if (!from_uvs) {
                        vertex = vertex.firstColocal();
                    }

                    faceIndices.Add(unifiedMeshIndices[vertex.id]);
                }

                m_unifiedMesh.addFace(faceIndices);
            }


            m_chartMesh.linkBoundary();
            m_unifiedMesh.linkBoundary();


            //exportMesh(m_unifiedMesh.ptr(), "debug_input.obj");

            if (m_unifiedMesh.splitBoundaryEdges()) {
                m_unifiedMesh = unifyVertices(m_unifiedMesh, m_chartToUnifiedMap, (uint)m_chartToUnifiedMap.Count);
            }

            //exportMesh(m_unifiedMesh.ptr(), "debug_split.obj");

            // Closing the holes is not always the best solution and does not fix all the problems.
            // We need to do some analysis of the holes and the genus to:
            // - Find cuts that reduce genus.
            // - Find cuts to connect holes.
            // - Use minimal spanning trees or seamster.
            if (!closeHoles()) {
                /*static int pieceCount = 0;
                StringBuilder fileName;
                fileName.format("debug_hole_%d.obj", pieceCount++);
                exportMesh(m_unifiedMesh.ptr(), fileName.str());*/
            }

            if (!isTriangularMesh(m_unifiedMesh)) {
                m_unifiedMesh = triangulate(m_unifiedMesh);
            }

            //exportMesh(m_unifiedMesh.ptr(), "debug_triangulated.obj");

            if (from_uvs) {
                // @@ Hack!
                // The other way to do this is to traverse the original vertices, lookup the unified/chart vertex index using the *MeshIndices arrays.
                // If not ~0, then write the position to the corresponding unified index location.

                // Restore vertex positions in UV chart.
                var vertex_count = m_unifiedMesh.vertexLength;
                for (var v = 0; v < vertex_count; v++) {
                    var vertex = m_unifiedMesh.vertexAt(v);
                    vertex.pos = vertex.nor;
                }

                vertex_count = m_chartMesh.vertexLength;
                for (var v = 0; v < vertex_count; v++) {
                    var vertex = m_chartMesh.vertexAt(v);
                    vertex.pos = vertex.nor;
                }
            }


            // Analyze chart topology.
            var topology = new MeshTopology(m_unifiedMesh);
            m_isDisk = topology.isDisk();

            // This is sometimes failing, when triangulate fails to add a triangle, it generates a hole in the mesh.
            //Debug.Assert(m_isDisk);

        }


        // Inverse of part1By1 - "delete" all odd-indexed bits
        public static uint compact1By1(uint x)
        {
            x &= 0x55555555;                  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
            x = (x ^ (x >> 1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
            x = (x ^ (x >> 2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
            x = (x ^ (x >> 4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
            x = (x ^ (x >> 8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
            return x;
        }

        // Inverse of part1By2 - "delete" all bits not at positions divisible by 3
        public static uint compact1By2(uint x)
        {
            x &= 0x09249249;                  // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
            x = (x ^ (x >> 2)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
            x = (x ^ (x >> 4)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
            x = (x ^ (x >> 8)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
            x = (x ^ (x >> 16)) & 0x000003ff; // x = ---- ---- ---- ---- ---- --98 7654 3210
            return x;
        }

        public static uint decodeMorton2X(uint code)
        {
            return compact1By1(code >> 0);
        }

        public static uint decodeMorton2Y(uint code)
        {
            return compact1By1(code >> 1);
        }

        public static uint decodeMorton3X(uint code)
        {
            return compact1By2(code >> 0);
        }

        public static uint decodeMorton3Y(uint code)
        {
            return compact1By2(code >> 1);
        }

        public static uint decodeMorton3Z(uint code)
        {
            return compact1By2(code >> 2);
        }

        public void buildVertexMap(ChartMesh originalMesh, List<uint> unchartedMaterialArray)
        {
            Debug.Assert(m_chartMesh == null && m_unifiedMesh == null);

            m_isVertexMapped = true;

            // Build face indices.
            m_faceArray.Clear();

            var meshFaceCount = originalMesh.faceLength;
            for (var f = 0; f < meshFaceCount; f++)
            {
                var face = originalMesh.faceAt(f);

                if (unchartedMaterialArray.Contains(face.material))
                {
                    m_faceArray.Add((uint)f);
                }
            }

            var faceCount = m_faceArray.Count;

            if (faceCount == 0)
            {
                return;
            }


            // @@ The chartMesh construction is basically the same as with regular charts, don't duplicate!

            var meshVertexCount = originalMesh.vertexLength;

            m_chartMesh = new ChartMesh();
            var chartMeshIndices = new uint[meshVertexCount];
            for (int i = 0; i < meshVertexCount; i++)
                chartMeshIndices[i] = unchecked((uint)~0u);

            // Vertex map mesh only has disconnected vertices.
            for (var f = 0; f < faceCount; f++)
            {
                var face = originalMesh.faceAt((int)m_faceArray[f]);
                Debug.Assert(face != null);

                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    var vertex = it.current().vertex;

                    if (chartMeshIndices[vertex.id] == unchecked((uint)~0u))
                    {
                        chartMeshIndices[vertex.id] = (uint)m_chartMesh.vertexLength;
                        m_chartToOriginalMap.Add(vertex.id);

                        var v = m_chartMesh.addVertex(vertex.pos);
                        v.nor = vertex.nor;
                        v.tex = vertex.tex; // @@ Not necessary.
                    }
                }
            }

            // @@ Link colocals using the original mesh canonical map? Build canonical map on the fly? Do we need to link colocals at all for this?
            //m_chartMesh.linkColocals();

            var faceIndices = new List<uint>(7);

            // Add faces.
            for (var f = 0; f < faceCount; f++)
            {
                var face = originalMesh.faceAt((int)m_faceArray[f]);
                Debug.Assert(face != null);

                faceIndices.Clear();

                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    var vertex = it.current().vertex;
                    Debug.Assert(vertex != null);
                    Debug.Assert(chartMeshIndices[vertex.id] != unchecked ((uint)~0u));

                    faceIndices.Add(chartMeshIndices[vertex.id]);
                }

                var new_face = m_chartMesh.addFace(faceIndices);
                Debug.Assert(new_face != null);
            }

            m_chartMesh.linkBoundary();


            var chartVertexCount = m_chartMesh.vertexLength;

            var bounds = new MinMaxAABB();
            if (chartVertexCount > 0)
            {
                {
                    var vertex = m_chartMesh.vertexAt(0);
                    bounds.Min = vertex.pos;
                    bounds.Max = vertex.pos;
                }

                for (var i = 1; i < chartVertexCount; i++)
                {
                    var vertex = m_chartMesh.vertexAt(i);
                    bounds.Min.x = Math.Min(bounds.Min.x, vertex.pos.x);
                    bounds.Min.y = Math.Min(bounds.Min.y, vertex.pos.y);
                    bounds.Min.z = Math.Min(bounds.Min.z, vertex.pos.z);

                    bounds.Max.x = Math.Min(bounds.Max.x, vertex.pos.x);
                    bounds.Max.y = Math.Min(bounds.Max.y, vertex.pos.y);
                    bounds.Max.z = Math.Min(bounds.Max.z, vertex.pos.z);
                }
            }

            var grid = new ProximityGrid();
            grid.init(bounds, chartVertexCount);

            for (var i = 0; i < chartVertexCount; i++)
            {
                var vertex = m_chartMesh.vertexAt(i);
                grid.add(vertex.pos, (uint)i);
            }

            uint texelCount = 0;

            const float positionThreshold = 0.01f;
            const float normalThreshold = 0.01f;

            uint verticesVisited = 0;
            uint cellsVisited = 0;

            var vertexIndexArray = new int[chartVertexCount];
            for (int i = 0; i < vertexIndexArray.Length; i++)
                vertexIndexArray[i] = -1; // Init all indices to -1.

            // Traverse vertices in morton order. @@ It may be more interesting to sort them based on orientation.
            var cellCodeCount = grid.mortonCount();
            for (var cellCode = 0; cellCode < cellCodeCount; cellCode++)
            {
                var cell = grid.mortonIndex((uint)cellCode);
                if (cell < 0) continue;

                cellsVisited++;

                var indexArray = grid.cellArray[cell].indexArray;

                var neighbors = new List<uint>();
                for (int i = 0; i < indexArray.Count; i++)
                {
                    var idx = indexArray[i];
                    var vertex = m_chartMesh.vertexAt((int)idx);

                    Debug.Assert(vertexIndexArray[idx] == -1);

                    neighbors.Clear();
                    grid.gather(vertex.pos, positionThreshold, neighbors);

                    // Compare against all nearby vertices, cluster greedily.
                    for (int j = 0; j < neighbors.Count; j++)
                    {
                        var otherIdx = neighbors[j];

                        if (vertexIndexArray[otherIdx] != -1)
                        {
                            var otherVertex = m_chartMesh.vertexAt((int)otherIdx);

                            if ((vertex.pos - otherVertex.pos).magnitude < positionThreshold &&
                                (vertex.nor - otherVertex.nor).magnitude < normalThreshold)
                            {
                                vertexIndexArray[idx] = vertexIndexArray[otherIdx];
                                break;
                            }
                        }
                    }

                    // If index not assigned, assign new one.
                    if (vertexIndexArray[idx] == -1)
                    {
                        var val = texelCount++;
                        vertexIndexArray[idx] = (int)val;
                    }

                    verticesVisited++;
                }
            }

            Debug.Assert(cellsVisited == grid.cellArray.Length);
            Debug.Assert(verticesVisited == chartVertexCount);

            vertexMapWidth = (uint)Mathf.CeilToInt(Mathf.Sqrt((float)(texelCount)));
            vertexMapWidth = (uint)((vertexMapWidth + 3) & ~3);                         // Width aligned to 4.
            vertexMapHeight = vertexMapWidth == 0 ? 0 : (texelCount + vertexMapWidth - 1) / vertexMapWidth;
            //vertexMapHeight = (vertexMapHeight + 3) & ~3;                             // Height aligned to 4.
            Debug.Assert(vertexMapWidth >= vertexMapHeight);

            Debug.Log($"Reduced vertex count from {chartVertexCount} to {texelCount}.");

            // Lay down the clustered vertices in morton order.

            var texelCodes = new uint[texelCount];

            // For each texel, assign one morton code.
            {
                uint texelCode = 0;
                for (uint i = 0; i < texelCount; i++)
                {
                    uint x, y;
                    do
                    {
                        x = decodeMorton2X(texelCode);
                        y = decodeMorton2Y(texelCode);
                        texelCode++;
                    } while (x >= (uint)(vertexMapWidth) || y >= (uint)(vertexMapHeight));

                    texelCodes[i] = texelCode - 1;
                }
            }

            for (var i = 0; i < chartVertexCount; i++)
            {
                var vertex = m_chartMesh.vertexAt(i);

                int idx = vertexIndexArray[i];
                if (idx != -1)
                {
                    uint texelCode = texelCodes[idx];
                    uint x = decodeMorton2X(texelCode);
                    uint y = decodeMorton2Y(texelCode);

                    vertex.tex.x = (float)(x);
                    vertex.tex.y = (float)(y);
                }
            }
        }
        
        public bool isDisk() { return m_isDisk; }
        public bool isVertexMapped() { return m_isVertexMapped; }

        public int vertexLength { get { return m_chartMesh.vertexLength; } }
        public int colocalVertexLength { get { return m_unifiedMesh.vertexLength; } }

        public int faceLength { get { return m_faceArray.Count; } }
        public uint faceAt(int i) { return m_faceArray[i]; }

        public ChartMesh chartMesh() { return m_chartMesh; }
        public ChartMesh unifiedMesh() { return m_unifiedMesh; }

        //uint vertexIndex(int i) { return m_vertexIndexArray[i]; }

        public uint mapChartVertexToOriginalVertex(int i) { return m_chartToOriginalMap[i]; }
        public uint mapChartVertexToUnifiedVertex(int i) { return m_chartToUnifiedMap[i]; }

        public List<uint> faceArray() { return m_faceArray; }

            
        float computeSurfaceArea(ChartMesh mesh)
        {
            float area = 0;

            for (var it = mesh.faces(); !it.isDone(); it.advance())
            {
                var face = it.current();
                area += face.area();
            }
            Debug.Assert(area >= 0);

            return area;
        }

        float computeParametricArea(ChartMesh mesh)
        {
            float area = 0;

            for (var it = mesh.faces(); !it.isDone(); it.advance())
            {
                var face = it.current();
                area += face.parametricArea();
            }

            return area;
        }


        public float computeSurfaceArea()
        {
            return computeSurfaceArea(m_chartMesh) * scale;
        }
        
        public float computeParametricArea()
        {
            // This only makes sense in parameterized meshes.
            Debug.Assert(m_isDisk);
            Debug.Assert(!m_isVertexMapped);

            return computeParametricArea(m_chartMesh);
        }
        
        public Vector2 computeParametricBounds()
        {
            // This only makes sense in parameterized meshes.
            Debug.Assert(m_isDisk);
            Debug.Assert(!m_isVertexMapped);

            var vertexCount = m_chartMesh.vertexLength;
            if (vertexCount == 0)
                return Vector2.zero;

            var min = m_chartMesh.vertexAt(0).tex;
            var max = min;

            for (int v = 1; v < vertexCount; v++)
            {
                var vertex = m_chartMesh.vertexAt(v);
                min.x = Math.Min(vertex.tex.x, min.x);
                min.y = Math.Min(vertex.tex.y, min.y);
                
                max.x = Math.Max(vertex.tex.x, max.x);
                max.y = Math.Max(vertex.tex.y, max.y);
            }

            return (max - min);
        }


        // Transfer parameterization from unified mesh to chart mesh.
        public void transferParameterization()
        {
            Debug.Assert(!m_isVertexMapped);

            var vertexCount = m_chartMesh.vertexLength;
            for (var v = 0; v < vertexCount; v++)
            {
                Vertex vertex = m_chartMesh.vertexAt(v);
                Vertex unifiedVertex = m_unifiedMesh.vertexAt((int)mapChartVertexToUnifiedVertex(v));
                vertex.tex = unifiedVertex.tex;
            }
        }

        //public void checkCharts(ChartMesh originalMesh);


        public float scale = 1.0f;
        public uint vertexMapWidth;
        public uint vertexMapHeight;
        public bool blockAligned = true;


        //private bool closeLoop(uint start, ref Edge[] loop);

        // Chart mesh.
        ChartMesh m_chartMesh;
        ChartMesh m_unifiedMesh;

        bool m_isDisk;
        bool m_isVertexMapped;

        // List of faces of the original mesh that belong to this chart.
        List<uint> m_faceArray = new List<uint>();

        // Map vertices of the chart mesh to vertices of the original mesh.
        List<uint> m_chartToOriginalMap = new List<uint>();

        List<uint> m_chartToUnifiedMap = new List<uint>();
    };
    
    // Estimate quality of existing parameterization.
    class ParameterizationQuality
    {
        public ParameterizationQuality()
        {
            m_totalTriangleCount = 0;
            m_flippedTriangleCount = 0;
            m_zeroAreaTriangleCount = 0;

            m_parametricArea = 0.0f;
            m_geometricArea = 0.0f;

            m_stretchMetric = 0.0f;
            m_maxStretchMetric = 0.0f;

            m_conformalMetric = 0.0f;
            m_authalicMetric = 0.0f;
        }

        public ParameterizationQuality(ChartMesh mesh)
        {
            Debug.Assert(mesh != null);

            m_totalTriangleCount = 0;
            m_flippedTriangleCount = 0;
            m_zeroAreaTriangleCount = 0;

            m_parametricArea = 0.0f;
            m_geometricArea = 0.0f;

            m_stretchMetric = 0.0f;
            m_maxStretchMetric = 0.0f;

            m_conformalMetric = 0.0f;
            m_authalicMetric = 0.0f;

            var faceCount = mesh.faceLength;
            for (var f = 0; f < faceCount; f++)
            {
                var face = mesh.faceAt(f);
                Vertex vertex0 = null;

                Vector3[] p = new Vector3[3];
                Vector2[] t = new Vector2[3];

                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    var edge = it.current();

                    if (vertex0 == null)
                    {
                        vertex0 = edge.vertex;

                        p[0] = vertex0.pos;
                        t[0] = vertex0.tex;
                    } else if (edge.to() != vertex0)
                    {
                        p[1] = edge.from().pos;
                        p[2] = edge.to().pos;
                        t[1] = edge.from().tex;
                        t[2] = edge.to().tex;

                        processTriangle(p, t);
                    }
                }
            }

            if (m_flippedTriangleCount + m_zeroAreaTriangleCount == faceCount)
            {
                // If all triangles are flipped, then none is.
                m_flippedTriangleCount = 0;
            }

            Debug.Assert(AtlasPacker.isFinite(m_parametricArea) && m_parametricArea >= 0);
            Debug.Assert(AtlasPacker.isFinite(m_geometricArea) && m_geometricArea >= 0);
            Debug.Assert(AtlasPacker.isFinite(m_stretchMetric));
            Debug.Assert(AtlasPacker.isFinite(m_maxStretchMetric));
            Debug.Assert(AtlasPacker.isFinite(m_conformalMetric));
            Debug.Assert(AtlasPacker.isFinite(m_authalicMetric));
        }

        public bool isValid()
        {
            return m_flippedTriangleCount == 0; // @@ Does not test for self-overlaps.
        }

        public float rmsStretchMetric()
        {
            if (m_geometricArea == 0) return 0.0f;
            float normFactor = Mathf.Sqrt(m_parametricArea / m_geometricArea);
            return Mathf.Sqrt(m_stretchMetric / m_geometricArea) * normFactor;
        }

        public float maxStretchMetric()
        {
            if (m_geometricArea == 0) return 0.0f;
            float normFactor = Mathf.Sqrt(m_parametricArea / m_geometricArea);
            return m_maxStretchMetric * normFactor;
        }

        public float rmsConformalMetric()
        {
            if (m_geometricArea == 0) return 0.0f;
            return Mathf.Sqrt(m_conformalMetric / m_geometricArea);
        }

        public float maxAuthalicMetric()
        {
            if (m_geometricArea == 0) return 0.0f;
            return Mathf.Sqrt(m_authalicMetric / m_geometricArea);
        }

        public static ParameterizationQuality operator + (ParameterizationQuality left, ParameterizationQuality right)
        {
            return new ParameterizationQuality
            {
                m_totalTriangleCount = left.m_totalTriangleCount + right.m_totalTriangleCount,
                m_flippedTriangleCount = left.m_flippedTriangleCount + right.m_flippedTriangleCount,
                m_zeroAreaTriangleCount = left.m_zeroAreaTriangleCount + right.m_zeroAreaTriangleCount,

                m_parametricArea = left.m_parametricArea + right.m_parametricArea,
                m_geometricArea = left.m_geometricArea + right.m_geometricArea,

                m_stretchMetric = left.m_stretchMetric + right.m_stretchMetric,
                m_maxStretchMetric = Math.Max(left.m_maxStretchMetric, right.m_maxStretchMetric),

                m_conformalMetric = left.m_conformalMetric + right.m_conformalMetric,
                m_authalicMetric = left.m_authalicMetric + right.m_authalicMetric
            };
        }

    
        void processTriangle(Vector3[] q/*[3]*/, Vector2[] p/*[3]*/)
        {
            m_totalTriangleCount++;

            // Evaluate texture stretch metric. See:
            // - "Texture Mapping Progressive Meshes", Sander, Snyder, Gortler & Hoppe
            // - "Mesh Parameterization: Theory and Practice", Siggraph'07 Course Notes, Hormann, Levy & Sheffer.

            float t1 = p[0].x;
            float s1 = p[0].y;
            float t2 = p[1].x;
            float s2 = p[1].y;
            float t3 = p[2].x;
            float s3 = p[2].y;

            float geometricArea = (Vector3.Cross(q[1] - q[0], q[2] - q[0])).magnitude / 2;
            float parametricArea = ((s2 - s1)*(t3 - t1) - (s3 - s1)*(t2 - t1)) / 2;
    
            if (Face.isZero(parametricArea))
            {
                m_zeroAreaTriangleCount++;
                return;
            }

            Vector3 Ss = (q[0] * (t2- t3) + q[1] * (t3 - t1) + q[2] * (t1 - t2)) / (2 * parametricArea);
            Vector3 St = (q[0] * (s3- s2) + q[1] * (s1 - s3) + q[2] * (s2 - s1)) / (2 * parametricArea);

            float a = Vector3.Dot(Ss, Ss); // E
            float b = Vector3.Dot(Ss, St); // F
            float c = Vector3.Dot(St, St); // G

            // Compute eigen-values of the first fundamental form:
            float sigma1 = Mathf.Sqrt(0.5f * Mathf.Max(0.0f, a + c - Mathf.Sqrt(AtlasPacker.square(a - c) + 4 * AtlasPacker.square(b)))); // gamma uppercase, min eigenvalue.
            float sigma2 = Mathf.Sqrt(0.5f * Mathf.Max(0.0f, a + c + Mathf.Sqrt(AtlasPacker.square(a - c) + 4 * AtlasPacker.square(b)))); // gamma lowercase, max eigenvalue.
            Debug.Assert(sigma2 >= sigma1);

            // isometric: sigma1 = sigma2 = 1
            // conformal: sigma1 / sigma2 = 1
            // authalic: sigma1 * sigma2 = 1

            float rmsStretch = Mathf.Sqrt((a + c) * 0.5f);
            float rmsStretch2 = Mathf.Sqrt((AtlasPacker.square(sigma1) + AtlasPacker.square(sigma2)) * 0.5f);
            Debug.Assert(AtlasPacker.equal(rmsStretch, rmsStretch2, 0.01f));

            if (parametricArea < 0.0f)
            {
                // Count flipped triangles.
                m_flippedTriangleCount++;

                parametricArea = Mathf.Abs(parametricArea);
            }

            m_stretchMetric += AtlasPacker.square(rmsStretch) * geometricArea;
            m_maxStretchMetric = Mathf.Max(m_maxStretchMetric, sigma2);

            if (!Face.isZero(sigma1, 0.000001f)) {
                // sigma1 is zero when geometricArea is zero.
                m_conformalMetric += (sigma2 / sigma1) * geometricArea;
            }
            m_authalicMetric += (sigma1 * sigma2) * geometricArea;

            // Accumulate total areas.
            m_geometricArea += geometricArea;
            m_parametricArea += parametricArea;


            //triangleConformalEnergy(q, p);
        }

        uint m_totalTriangleCount;
        uint m_flippedTriangleCount;
        uint m_zeroAreaTriangleCount;

        float m_parametricArea;
        float m_geometricArea;

        float m_stretchMetric;
        float m_maxStretchMetric;

        float m_conformalMetric;
        float m_authalicMetric;

    };

    class MeshCharts
    {
        public MeshCharts(ChartMesh mesh)
        {
            m_mesh = mesh;
            m_totalVertexCount = 0;
            m_chartArray = new List<Chart>();
            m_chartVertexCountPrefixSum = new uint[0];
            m_faceChart = new uint[0];
            m_faceIndex = new uint[0];
        }


        public int chartLength { get { return m_chartArray.Count; } }
        public int vertexLength { get { return m_totalVertexCount; } }

        public Chart chartAt(int i) { return m_chartArray[i]; }

        //public void computeVertexMap(ref uint[] unchartedMaterialArray);

        //public void parameterizeCharts(bool preserve_uvs, bool pinned_boundary);

        public uint faceChartAt(uint i) { return m_faceChart[i]; }
        public uint faceIndexWithinChartAt(uint i) { return m_faceIndex[i]; }

        public uint vertexCountBeforeChartAt(uint i) { return m_chartVertexCountPrefixSum[i]; }



        // Extract the charts of the input mesh.
        public void extractCharts(bool ensure_disk_charts)
        {
            var faceCount = m_mesh.faceLength;

            int first = 0;
            var queue = new List<uint>(faceCount);
            var boundaries = new List<uint>(faceCount);

            var bitFlags = new BitArray((uint)faceCount);
            bitFlags.clearAll();

            for (uint f = 0; f < faceCount; f++)
            {
                if (bitFlags.bitAt(f) == false)
                {
                    // Two trials:
                    // - Why are we doing things this way? If we have two charts that each happen to a boundary edge with the same uvs, then the region growing algorithm
                    //   consider them to be a single chart. If we respect all seams, then normal seams will be considered chart boundaries and will have too many seams.
                    //   This solution does not fix the problem entirely, but it provides a workaround for the case that growing the charts too aggressively results in 
                    //   non-disk chart topologies. Ideally we should preserve the original connectivity in the input mesh.
                    for (uint t = 0; t < 2; t++)
                    {

                        // Start new patch. Reset queue.
                        first = 0;
                        queue.Clear();
                        //boundaries.clear();
                        queue.Add(f);
                        bitFlags.setBitAt(f);

                        while (first != queue.Count)
                        {
                            var face = m_mesh.faceAt((int)queue[first]);

                            // Visit face neighbors of queue[first]
                            for (var it = face.edges(); !it.isDone(); it.advance())
                            {
                                var edge = it.current();
                                Debug.Assert(edge.pair != null);

                                bool is_boundary = edge.isBoundary();
                                bool is_seam = edge.isSeam();
                                bool is_tex_seam = edge.vertex.tex != edge.pair.next.vertex.tex || edge.next.vertex.tex != edge.pair.vertex.tex;

                                // During the first try we stop growing chart at texture seams only.
                                // On the second try we stop at any seam.
                                if (t == 0) is_seam = is_tex_seam;

                                if (!is_boundary && !is_seam)
                                {
                                    var neighborFace = edge.pair.face;
                                    Debug.Assert(neighborFace != null);

                                    if (bitFlags.bitAt(neighborFace.id) == false)
                                    {
                                        queue.Add(neighborFace.id);
                                        bitFlags.setBitAt(neighborFace.id);
                                    }
                                } else
                                {
                                    //boundaries.Add(edge.id);
                                }
                            }

                            first++;
                        }

                        var chart = new Chart();
                        chart.build(m_mesh, queue, /*from_uvs=*/true);

                        // If not a disk on our first try, delete this chart and try again.
                        if (t == 0 && !chart.isDisk())
                        {
                            foreach (var q in queue)
                                bitFlags.clearBitAt(q);
                            continue;
                        }

                        m_chartArray.Add(chart);
                        break;
                    }
                }
            }

            var chartCount = m_chartArray.Count;

            // Print summary.
            int disk_count = 0;
            for (var i = 0; i < chartCount; i++)
            {
                var chart = m_chartArray[i];
                disk_count += chart.isDisk() ? 1 : 0;
            }
            //Debug.Log("--- Extracted %d charts (%d with disk topology).\n", m_chartArray.Count, disk_count);

            // Build face indices.
            Array.Resize(ref m_faceChart, m_mesh.faceLength);
            Array.Resize(ref m_faceIndex, m_mesh.faceLength);

            for (var i = 0; i < chartCount; i++)
            {
                var chart = m_chartArray[i];

                var chartFaceCount = chart.faceLength;
                for (var f = 0; f < chartFaceCount; f++)
                {
                    uint idx = chart.faceAt(f);
                    m_faceChart[idx] = (uint)i;
                    m_faceIndex[idx] = (uint)f;
                }
            }

            // Build an exclusive prefix sum of the chart vertex counts.
            Array.Resize(ref m_chartVertexCountPrefixSum, chartCount);

            if (chartCount > 0)
            {
                m_chartVertexCountPrefixSum[0] = 0;

                for (var i = 1; i < chartCount; i++)
                {
                    var chart = m_chartArray[i - 1];
                    m_chartVertexCountPrefixSum[i] = (uint)(m_chartVertexCountPrefixSum[i - 1] + (uint)chart.vertexLength);
                }

                m_totalVertexCount = (int)(m_chartVertexCountPrefixSum[chartCount - 1] + (uint)(m_chartArray[chartCount - 1].vertexLength));
            } else
            {
                m_totalVertexCount = 0;
            }
        }


        // Compute charts using a simple segmentation algorithm.
        public void computeCharts(SegmentationSettings settings, List<uint> unchartedMaterialArray)
        {
            Chart vertexMap = null;

            if (unchartedMaterialArray.Count != 0)
            {
                vertexMap = new Chart();
                vertexMap.buildVertexMap(m_mesh, unchartedMaterialArray);

                if (vertexMap.faceLength == 0) {
                    vertexMap = null;
                }
            }
    

            var builder = new AtlasBuilder(m_mesh);

            if (vertexMap != null) {
                // Mark faces that do not need to be charted.
                builder.markUnchartedFaces(vertexMap.faceArray());

                m_chartArray.Add(vertexMap);
            }

            if (builder.facesLeft != 0) {

                // Tweak these values:
                const float maxThreshold = 2;
                const uint growFaceCount = 32;
                const uint maxIterations = 4;
        
                builder.settings = settings;

                //builder.settings.proxyFitMetricWeight *= 0.75; // relax proxy fit weight during initial seed placement.
                //builder.settings.roundnessMetricWeight = 0;
                //builder.settings.straightnessMetricWeight = 0;

                // This seems a reasonable estimate.
                uint maxSeedCount = Math.Max(6U, builder.facesLeft);

                // Create initial charts greedely.
                //Debug.Log("### Placing seeds\n");
                builder.placeSeeds(maxThreshold, maxSeedCount);
                //Debug.Log("###   Placed %d seeds (max = %d)\n", builder.chartCount(), maxSeedCount);

                builder.updateProxies();

                builder.mergeCharts();

                //Debug.Log("### Relocating seeds\n");
                builder.relocateSeeds();

                //Debug.Log("### Reset charts\n");
                builder.resetCharts();

                if (vertexMap != null) {
                    builder.markUnchartedFaces(vertexMap.faceArray());
                }

                builder.settings = settings;

                //Debug.Log("### Growing charts\n");

                // Restart process growing charts in parallel.
                uint iteration = 0;
                while (true)
                {
                    if (!builder.growCharts(maxThreshold, growFaceCount))
                    {
                        //Debug.Log("### Can't grow anymore\n");

                        // If charts cannot grow more: fill holes, merge charts, relocate seeds and start new iteration.

                        //Debug.Log("### Filling holes\n");
                        builder.fillHoles(maxThreshold);
                        //Debug.Log("###   Using %d charts now\n", builder.chartCount());

                        builder.updateProxies();

                        //Debug.Log("### Merging charts\n");
                        builder.mergeCharts();
                        //Debug.Log("###   Using %d charts now\n", builder.chartCount());

                        //Debug.Log("### Reseeding\n");
                        if (!builder.relocateSeeds())
                        {
                            //Debug.Log("### Cannot relocate seeds anymore\n");

                            // Done!
                            break;
                        }

                        if (iteration == maxIterations)
                        {
                            //Debug.Log("### Reached iteration limit\n");
                            break;
                        }
                        iteration++;

                        //Debug.Log("### Reset charts\n");
                        builder.resetCharts();

                        if (vertexMap != null) {
                            builder.markUnchartedFaces(vertexMap.faceArray());
                        }

                        //Debug.Log("### Growing charts\n");
                    }
                };

                // Make sure no holes are left!
                Debug.Assert(builder.facesLeft == 0);

                var chartCount = builder.chartArray.Count;
                for (var i = 0; i < chartCount; i++)
                {
                    var chart = new Chart();
                    m_chartArray.Add(chart);

                    chart.build(m_mesh, builder.chartFaces((uint)i));
                }
            }

            {
                var chartCount = m_chartArray.Count;

                // Build face indices.
                Array.Resize(ref m_faceChart, m_mesh.faceLength);
                Array.Resize(ref m_faceIndex, m_mesh.faceLength);

                for (var i = 0; i < chartCount; i++)
                {
                    var chart = m_chartArray[i];

                    var faceCount = chart.faceLength;
                    for (var f = 0; f < faceCount; f++)
                    {
                        uint idx = chart.faceAt(f);
                        m_faceChart[idx] = (uint)i;
                        m_faceIndex[idx] = (uint)f;
                    }
                }

                // Build an exclusive prefix sum of the chart vertex counts.
                Array.Resize(ref m_chartVertexCountPrefixSum, chartCount);

                if (chartCount > 0)
                {
                    m_chartVertexCountPrefixSum[0] = 0;

                    for (var i = 1; i < chartCount; i++)
                    {
                        var chart = m_chartArray[i - 1];
                        m_chartVertexCountPrefixSum[i] = (uint)(m_chartVertexCountPrefixSum[i - 1] + chart.vertexLength);
                    }

                    m_totalVertexCount = (int)(m_chartVertexCountPrefixSum[chartCount - 1] + m_chartArray[chartCount - 1].vertexLength);
                } else
                {
                    m_totalVertexCount = 0;
                }
            }
        }


        void computeSingleFaceMap(ChartMesh mesh)
        {
            Debug.Assert(mesh != null);
            Debug.Assert(mesh.faceLength == 1);

            Face face = mesh.faceAt(0);
            Debug.Assert(face != null);

            Vector3 p0 = face.edge.from().pos;
            Vector3 p1 = face.edge.to().pos;

            Vector3 X = Face.normalizeSafe(p1 - p0, Vector3.zero, 0.0f);
            Vector3 Z = face.normal();
            Vector3 Y = Face.normalizeSafe(Vector3.Cross(Z, X), Vector3.zero, 0.0f);

            uint i = 0;
            for (var it = face.edges(); !it.isDone(); it.advance(), i++)
            {
                Vertex vertex = it.vertex();
                Debug.Assert(vertex != null);

                if (i == 0)
                {
                    vertex.tex = Vector2.zero;
                } else
                {
                    Vector3 pn = vertex.pos;

                    float xn = Vector3.Dot((pn - p0), X);
                    float yn = Vector3.Dot((pn - p0), Y);

                    vertex.tex = new Vector2(xn, yn);
                }
            }
        }

        
        int countMeshTriangles(ChartMesh mesh)
        {
            var faceCount = mesh.faceLength;

            var triangleCount = 0;

            for (var f = 0; f < faceCount; f++)
            {
                var face = mesh.faceAt(f);
        
                var edgeCount = face.edgeLength;
                Debug.Assert(edgeCount > 2);

                triangleCount += edgeCount - 2;
            }

            return triangleCount;
        }
        
        int countBoundaryVertices(ChartMesh mesh)
        {
            var boundaryVertexCount = 0;
    
            var vertexCount = mesh.vertexLength;
            for (var v = 0; v < vertexCount; v++) {
                var vertex = mesh.vertexAt(v);
                if (vertex.isBoundary()) boundaryVertexCount += 1;
            }

            return boundaryVertexCount;
        }
        

        // Fast sweep in 3 directions
        static bool findApproximateDiameterVertices(ChartMesh mesh, out Vertex a, out Vertex b)
        {
            Debug.Assert(mesh != null);

            var vertexCount = mesh.vertexLength;

            var minVertex = new Vertex[3];
            var maxVertex = new Vertex[3];

            minVertex[0] = minVertex[1] = minVertex[2] = null;
            maxVertex[0] = maxVertex[1] = maxVertex[2] = null;

            for (var v = 1; v < vertexCount; v++)
            {
                var vertex = mesh.vertexAt(v);
                Debug.Assert(vertex != null);

                if (vertex.isBoundary())
                {
                    minVertex[0] = minVertex[1] = minVertex[2] = vertex;
                    maxVertex[0] = maxVertex[1] = maxVertex[2] = vertex;
                    break;
                }
            }

            if (minVertex[0] == null)
            {
                // Input mesh has not boundaries.
                a = null;
                b = null;
                return false;
            }

            for (var v = 1; v < vertexCount; v++)
            {
                var vertex = mesh.vertexAt(v);
                Debug.Assert(vertex != null);

                if (!vertex.isBoundary())
                {
                    // Skip interior vertices.
                    continue;
                }

                if (vertex.pos.x < minVertex[0].pos.x) minVertex[0] = vertex;
                else if (vertex.pos.x > maxVertex[0].pos.x) maxVertex[0] = vertex;

                if (vertex.pos.y < minVertex[1].pos.y) minVertex[1] = vertex;
                else if (vertex.pos.y > maxVertex[1].pos.y) maxVertex[1] = vertex;

                if (vertex.pos.z < minVertex[2].pos.z) minVertex[2] = vertex;
                else if (vertex.pos.z > maxVertex[2].pos.z) maxVertex[2] = vertex;
            }

            var lengths = new float[3];
            for (int i = 0; i < 3; i++)
            {
                lengths[i] = (minVertex[i].pos - maxVertex[i].pos).magnitude;
            }

            if (lengths[0] > lengths[1] && lengths[0] > lengths[2])
            {
                a = minVertex[0];
                b = maxVertex[0];
            } else if (lengths[1] > lengths[2])
            {
                a = minVertex[1];
                b = maxVertex[1];
            } else
            {
                a = minVertex[2];
                b = maxVertex[2];
            }

            return true;
        }

        public enum Transpose
        {
            NoTransposed = 0,
            Transposed = 1
        };

        public static void mult(Transpose TM, SparseMatrix M, FullVector x, FullVector y)
        {
            var w = M.width();
            var h = M.height();

            if (TM == Transpose.Transposed)
            {
                Debug.Assert( h == x.dimension() );
                Debug.Assert( w == y.dimension() );

                y.fill(0.0f);

                for (var i = 0; i < h; i++)
                {
                    M.madRow((uint)i, x[i], y);
                }
            }
            else
            {
                Debug.Assert( w == x.dimension() );
                Debug.Assert( h == y.dimension() );

                for (var i = 0; i < h; i++)
                {
                    y[i] = M.dotRow((uint)i, x);
                }
            }
        }

        // y = M * x
        public static void mult(SparseMatrix M, FullVector x, FullVector y)
        {
            mult(Transpose.NoTransposed, M, x, y);
        }
        
        public static void mult(Transpose TA, SparseMatrix A, Transpose TB, SparseMatrix B, SparseMatrix C)
        {
            sgemm(1.0f, TA, A, TB, B, 0.0f, C);
        }

        // C = alpha*A*B + beta*C
        public static void sgemm(float alpha, SparseMatrix A, SparseMatrix B, float beta, SparseMatrix C)
        {
            sgemm(alpha, Transpose.NoTransposed, A, Transpose.NoTransposed, B, beta, C);
        }


        // dot y-row of A by x-column of B
        public static float dotRowColumn(int y, SparseMatrix A, int x, SparseMatrix B)
        {
            var row = A.getRow((uint)y);

            var count = row.Count;

        #if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < count; i++)
            {
                const SparseMatrix.Coefficient & c = row[i];
                kahan.add(c.v * B.getCoefficient(x, c.x));
            }
            return kahan.sum();
        #else
            float sum = 0.0f;
            for (var i = 0; i < count; i++)
            {
                var c = row[i];
                sum += c.v * B.getCoefficient((uint)x, c.x);
            }
            return sum;
        #endif
        }

        // dot y-row of A by x-row of B
        public static float dotRowRow(int y, SparseMatrix A, int x, SparseMatrix B)
        {
            var row = A.getRow((uint)y);

            var count = row.Count;

        #if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < count; i++)
            {
                const SparseMatrix.Coefficient & c = row[i];
                kahan.add(c.v * B.getCoefficient(c.x, x));
            }
            return kahan.sum();
        #else
            float sum = 0.0f;
            for (var i = 0; i < count; i++)
            { 
                var c = row[i];
                sum += c.v * B.getCoefficient(c.x, (uint)x);
            }
            return sum;
        #endif
        }

        // dot y-column of A by x-column of B
        public static float dotColumnColumn(int y, SparseMatrix A, int x, SparseMatrix B)
        {
            Debug.Assert(A.height() == B.height());

            var h = A.height();

        #if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < h; i++)
            {
                kahan.add(A.getCoefficient(y, i) * B.getCoefficient(x, i));
            }
            return kahan.sum();
        #else
            float sum = 0.0f;
            for (var i = 0; i < h; i++)
            {
                sum += A.getCoefficient((uint)y, (uint)i) * B.getCoefficient((uint)x, (uint)i);
            }
            return sum;
        #endif
        }


        public static void sgemm(float alpha, Transpose TA, SparseMatrix A, Transpose TB, SparseMatrix B, float beta, SparseMatrix C)
        {
            var w = C.width();
            var h = C.height();

            uint aw = (TA == Transpose.NoTransposed) ? A.width() : A.height();
            uint ah = (TA == Transpose.NoTransposed) ? A.height() : A.width();
            uint bw = (TB == Transpose.NoTransposed) ? B.width() : B.height();
            uint bh = (TB == Transpose.NoTransposed) ? B.height() : B.width();

            Debug.Assert(aw == bh);
            Debug.Assert(bw == ah);
            Debug.Assert(w == bw);
            Debug.Assert(h == ah);


            for (var y = 0; y < h; y++)
            {
                for (var x = 0; x < w; x++)
                {
                    float c = beta * C.getCoefficient((uint)x, (uint)y);

                    if (TA == Transpose.NoTransposed && TB == Transpose.NoTransposed)
                    {
                        // dot y-row of A by x-column of B.
                        c += alpha * dotRowColumn(y, A, x, B);
                    }
                    else if (TA == Transpose.Transposed && TB == Transpose.Transposed)
                    {
                        // dot y-column of A by x-row of B.
                        c += alpha * dotRowColumn(x, B, y, A);
                    }
                    else if (TA == Transpose.Transposed && TB == Transpose.NoTransposed)
                    {
                        // dot y-column of A by x-column of B.
                        c += alpha * dotColumnColumn(y, A, x, B);
                    }
                    else if (TA == Transpose.NoTransposed && TB == Transpose.Transposed)
                    {
                        // dot y-row of A by x-row of B.
                        c += alpha * dotRowRow(y, A, x, B);
                    }

                    C.setCoefficient((uint)x, (uint)y, c);
                }
            }
        }

        // C = A * B
        public static void mult(SparseMatrix A, SparseMatrix B, SparseMatrix C)
        {
            mult(Transpose.NoTransposed, A, Transpose.NoTransposed, B, C);
        }

        public static void transpose(SparseMatrix A, SparseMatrix B)
        {
            Debug.Assert(A.width() == B.height());
            Debug.Assert(B.width() == A.height());

            var w = A.width();
            for (var x = 0; x < w; x++)
            {
                B.clearRow((uint)x);
            }

            var h = A.height();
            for (var y = 0; y < h; y++)
            {
                var row = A.getRow((uint)y);

                var count = row.Count;
                for (var i = 0; i < count; i++)
                {
                    var c = row[i];
                    Debug.Assert(c.x < w);

                    B.setCoefficient((uint)y, c.x, c.v);
                }
            }
        }


        static void copy(FullVector x, FullVector y)
        {
            Debug.Assert(x.dimension() == y.dimension());

            var dim = x.dimension();
            for (var i = 0; i < dim; i++)
            {
                y[i] = x[i];
            }
        }
        
        static float dot(FullVector x, FullVector y)
        {
            Debug.Assert(x.dimension() == y.dimension());

            var dim = x.dimension();

        #if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < dim; i++)
            {
                kahan.add(x[i] * y[i]);
            }
            return kahan.sum();
        #else
            float sum = 0;
            for (var i = 0; i < dim; i++)
            {
                sum += x[i] * y[i];
            }
            return sum;
        #endif
        }


        // y = alpha*A*x + beta*y
        static void sgemv(float alpha, SparseMatrix A, FullVector x, float beta, FullVector y)
        {
            sgemv(alpha, Transpose.NoTransposed, A, x, beta, y);
        }

        static void sgemv(float alpha, Transpose TA, SparseMatrix A, FullVector x, float beta, FullVector y)
        {
            var w = A.width();
            var h = A.height();

            if (TA == Transpose.Transposed)
            {
                Debug.Assert( h == x.dimension() );
                Debug.Assert( w == y.dimension() );

                for (var i = 0; i < h; i++)
                {
                    A.madRow((uint)i, alpha * x[i], y);
                }
            }
            else
            {
                Debug.Assert( w == x.dimension() );
                Debug.Assert( h == y.dimension() );

                for (var i = 0; i < h; i++)
                {
                    y[i] = alpha * A.dotRow((uint)i, x) + beta * y[i];
                }
            }
        }

        static void saxpy(float a, FullVector x, FullVector y)
        {
            Debug.Assert(x.dimension() == y.dimension());

            var dim = x.dimension();
            for (var i = 0; i < dim; i++)
            {
                y[i] += a * x[i];
            }
        }

        static void scal(float a, FullVector x)
        {
            var dim = x.dimension();
            for (var i = 0; i < dim; i++)
            {
                x[i] *= a;
            }
        }

        // Conjugate gradient with preconditioner.
        static bool ConjugateGradientSolver(JacobiPreconditioner preconditioner, SparseMatrix A, FullVector b, FullVector x, float epsilon)
        {
            Debug.Assert( A.isSquare() );
            Debug.Assert( A.width() == b.dimension() );
            Debug.Assert( A.width() == x.dimension() );

            int i = 0;
            var D = A.width();
            var i_max = 4 * D;   // Convergence should be linear, but in some cases, it's not.

            var r = new FullVector(D);    // residual
            var p = new FullVector(D);    // search direction
            var q = new FullVector(D);    // 
            var s = new FullVector(D);    // preconditioned
            float delta_0;
            float delta_old;
            float delta_new;
            float alpha;
            float beta;

            // r = b - A·x
            copy(b, r);
            sgemv(-1, A, x, 1, r);


            // p = M^-1 · r
            preconditioner.apply(r, p);
            //copy(r, p);


            delta_new = dot(r, p);
            delta_0 = delta_new;

            while (i < i_max && delta_new > epsilon*epsilon*delta_0)
            {
                i++;

                // q = A·p
                mult(A, p, q);

                // alpha = delta_new / p·q
                alpha = delta_new / dot(p, q);

                // x = alfa·p + x
                saxpy(alpha, p, x);

                if ((i & 31) == 0)  // recompute r after 32 steps
                {			
                    // r = b - A·x
                    copy(b, r);
                    sgemv(-1, A, x, 1, r);
                }
                else
                {
                    // r = r - alfa·q
                    saxpy(-alpha, q, r);
                }

                // s = M^-1 · r
                preconditioner.apply(r, s);
                //copy(r, s);

                delta_old = delta_new;
                delta_new = dot( r, s );

                beta = delta_new / delta_old;

                // p = s + beta·p
                scal(beta, p);
                saxpy(1, s, p);
            }

            return delta_new <= epsilon*epsilon*delta_0;
        }

        
        public static bool SymmetricSolver(SparseMatrix A, FullVector b, FullVector x, float epsilon/*1e-5f*/)
        {
            Debug.Assert(A.height() == A.width());
            Debug.Assert(A.height() == b.dimension());
            Debug.Assert(b.dimension() == x.dimension());

            var jacobi = new JacobiPreconditioner(A, true);
            return ConjugateGradientSolver(jacobi, A, b, x, epsilon);

            //return ConjugateGradientSolver(A, b, x, epsilon);
        }

        
        // Solve the symmetric system: At·A·x = At·b
        public static bool LeastSquaresSolver(SparseMatrix A, FullVector b, FullVector x, float epsilon/*1e-5f*/)
        {
            Debug.Assert(A.width() == x.dimension());
            Debug.Assert(A.height() == b.dimension());
            Debug.Assert(A.height() >= A.width()); // @@ If height == width we could solve it directly...

            var D = A.width();

            var At = new SparseMatrix(A.height(), A.width());
            transpose(A, At);

            var Atb = new FullVector(D);
            //mult(Transposed, A, b, Atb);
            mult(At, b, Atb);

            var AtA = new SparseMatrix(D);
            //mult(Transposed, A, NoTransposed, A, AtA);
            mult(At, A, AtA);

            return SymmetricSolver(AtA, Atb, x, epsilon);
        }


        // See section 10.4.3 in: Mesh Parameterization: Theory and Practice, Siggraph Course Notes, August 2007
        public static bool LeastSquaresSolver(SparseMatrix A, FullVector b, FullVector x, List<uint> lockedParameters, uint lockedCount, float epsilon/*= 1e-5f*/)
        {
            Debug.Assert(A.width() == x.dimension());
            Debug.Assert(A.height() == b.dimension());
            Debug.Assert(A.height() >= A.width() - lockedCount);

            // @@ This is not the most efficient way of building a system with reduced degrees of freedom. It would be faster to do it on the fly.

            var D = A.width() - lockedCount;
            Debug.Assert(D > 0);

            // Compute: b - Al * xl
            var b_Alxl = new FullVector(b);

            for (var y = 0; y < A.height(); y++)
            {
                var count = A.getRow((uint)y).Count;
                for (var e = 0; e < count; e++)
                {
                    var column = A.getRow((uint)y)[e].x;

                    bool isFree = true;
                    for (var i = 0; i < lockedCount; i++) 
                    {
                        isFree &= (lockedParameters[i] != column);
                    }

                    if (!isFree)
                    {
                        b_Alxl[y] -= x[(int)column] * A.getRow((uint)y)[e].v;
                    }
                }
            }

            // Remove locked columns from A.
            var  Af = new SparseMatrix(D, A.height());

            for (var y = 0; y < A.height(); y++)
            {
                var count = A.getRow((uint)y).Count;
                for (var e = 0; e < count; e++)
                {
                    uint column = A.getRow((uint)y)[e].x;
                    uint ix = column;

                    bool isFree = true;
                    for (var i = 0; i < lockedCount; i++) 
                    {
                        isFree &= (lockedParameters[i] != column);
                        if (column > lockedParameters[i]) ix--; // shift columns
                    }

                    if (isFree)
                    {
                        Af.setCoefficient(ix, (uint)y, A.getRow((uint)y)[e].v);
                    }
                }
            }

            // Remove elements from x
            var xf = new FullVector(D);

            for (int i = 0, j = 0; i < A.width(); i++)
            {
                bool isFree = true;
                for (var l = 0; l < lockedCount; l++) 
                {
                    isFree &= (lockedParameters[l] != i);
                }

                if (isFree)
                {
                    xf[j++] = x[i];
                }
            }

            // Solve reduced system.
            bool result = LeastSquaresSolver(Af, b_Alxl, xf, epsilon);

            // Copy results back to x.
            for (int i = 0, j = 0; i < A.width(); i++)
            {
                bool isFree = true;
                for (var l = 0; l < lockedCount; l++) 
                {
                    isFree &= (lockedParameters[l] != i);
                }

                if (isFree)
                {
                    x[i] = xf[j++];
                }
            }

            return result;
        }
        
        // Conformal relations from Brecht Van Lommel (based on ABF):

        static float vec_angle_cos(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            Vector3 d1 = v1 - v2;
            Vector3 d2 = v3 - v2;
            return Mathf.Clamp(Vector3.Dot(d1, d2) / (d1.magnitude * d2.magnitude), -1.0f, 1.0f);
        }

        static float vec_angle(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            float dot = vec_angle_cos(v1, v2, v3);
            return Mathf.Acos(dot);
        }

        static void triangle_angles(Vector3 v1, Vector3 v2, Vector3 v3, out float a1, out float a2, out float a3)
        {
            a1 = vec_angle(v3, v1, v2);
            a2 = vec_angle(v1, v2, v3);
            a3 = Mathf.PI - a2 - a1;
        }

        static void setup_abf_relations(SparseMatrix A, int row, Vertex v0, Vertex v1, Vertex v2)
        {
            var id0 = v0.id;
            var id1 = v1.id;
            var id2 = v2.id;

            var p0 = v0.pos;
            var p1 = v1.pos;
            var p2 = v2.pos;

            // @@ IC: Wouldn't it be more accurate to return cos and compute 1-cos^2?
            // It does indeed seem to be a little bit more robust.
            // @@ Need to revisit this more carefully!

            triangle_angles(p0, p1, p2, out var a0, out var a1, out var a2);

            float s0 = Mathf.Sin(a0);
            float s1 = Mathf.Sin(a1);
            float s2 = Mathf.Sin(a2);

            /*// Hack for degenerate triangles.
            if (equal(s0, 0) && equal(s1, 0) && equal(s2, 0)) {
                if (equal(a0, 0)) a0 += 0.001f;
                if (equal(a1, 0)) a1 += 0.001f;
                if (equal(a2, 0)) a2 += 0.001f;

                if (equal(a0, PI)) a0 = PI - a1 - a2;
                if (equal(a1, PI)) a1 = PI - a0 - a2;
                if (equal(a2, PI)) a2 = PI - a0 - a1;

                s0 = Mathf.Sin(a0);
                s1 = Mathf.Sin(a1);
                s2 = Mathf.Sin(a2);
            }*/

            if (s1 > s0 && s1 > s2)
            {
                AtlasPacker.swap(ref s1, ref s2);
                AtlasPacker.swap(ref s0, ref s1);

                AtlasPacker.swap(ref a1, ref a2);
                AtlasPacker.swap(ref a0, ref a1);

                AtlasPacker.swap(ref id1, ref id2);
                AtlasPacker.swap(ref id0, ref id1);
            }
            else if (s0 > s1 && s0 > s2)
            {
                AtlasPacker.swap(ref s0, ref s2);
                AtlasPacker.swap(ref s0, ref s1);

                AtlasPacker.swap(ref a0, ref a2);
                AtlasPacker.swap(ref a0, ref a1);

                AtlasPacker.swap(ref id0, ref id2);
                AtlasPacker.swap(ref id0, ref id1);
            }

            float c0 = Mathf.Cos(a0);

            float ratio = (s2 == 0.0f) ? 1.0f: s1/s2;
            float cosine = c0 * ratio;
            float sine = s0 * ratio;

            // Note  : 2*id + 0 --> u
            //         2*id + 1 --> v
            var u0_id = 2 * id0 + 0;
            var v0_id = 2 * id0 + 1;
            var u1_id = 2 * id1 + 0;
            var v1_id = 2 * id1 + 1;
            var u2_id = 2 * id2 + 0;
            var v2_id = 2 * id2 + 1;

            // Real part
            A.setCoefficient(u0_id, (uint)(2 * row + 0), cosine - 1.0f);
            A.setCoefficient(v0_id, (uint)(2 * row + 0), -sine);
            A.setCoefficient(u1_id, (uint)(2 * row + 0), -cosine);
            A.setCoefficient(v1_id, (uint)(2 * row + 0), sine);
            A.setCoefficient(u2_id, (uint)(2 * row + 0), 1);

            // Imaginary part
            A.setCoefficient(u0_id, (uint)(2 * row + 1), sine);
            A.setCoefficient(v0_id, (uint)(2 * row + 1), cosine - 1.0f);
            A.setCoefficient(u1_id, (uint)(2 * row + 1), -sine);
            A.setCoefficient(v1_id, (uint)(2 * row + 1), -cosine);
            A.setCoefficient(v2_id, (uint)(2 * row + 1), 1);
        }


        bool computeLeastSquaresConformalMap(ChartMesh mesh, bool pinned_boundaries/*=false*/)
        {
            Debug.Assert(mesh != null);

            // For this to work properly, mesh should not have colocals that have the same 
            // attributes, unless you want the vertices to actually have different texcoords.

            var vertexCount = mesh.vertexLength;
            var D = 2 * vertexCount;
            var N = 2 * countMeshTriangles(mesh);

            // N is the number of equations (one per triangle)
            // D is the number of variables (one per vertex).
            // P is the number of locked parameters.
            var P = 4;
            if (pinned_boundaries) P = 2 * countBoundaryVertices(mesh);

            if (N < D - P || D <= P)
            {
                return false;
            }

            var A = new SparseMatrix((uint)D, (uint)N);
            var b = new FullVector((uint)N);
            var x = new FullVector((uint)D);

            // Fill b:
            b.fill(0.0f);


            var lockedParameters = new List<uint>();

            // Fill x:
            if (!pinned_boundaries)
            {
                if (!findApproximateDiameterVertices(mesh, out var v0, out var v1))
                {
                    // Mesh has no boundaries.
                    return false;
                }
                if (v0.tex == v1.tex)
                {
                    // LSCM expects an existing parameterization.
                    return false;
                }

                // Pin two vertices:
                lockedParameters.Add(2 * v0.id + 0);
                lockedParameters.Add(2 * v0.id + 1);
                lockedParameters.Add(2 * v1.id + 0);
                lockedParameters.Add(2 * v1.id + 1);
            } else
            {
                // Pin all boundary vertices:
                for (var v = 0; v < vertexCount; v++)
                {
                    Vertex vertex = mesh.vertexAt(v);
                    if (vertex.isBoundary())
                    {
                        lockedParameters.Add(2 * vertex.id + 0);
                        lockedParameters.Add(2 * vertex.id + 1);
                    }
                }
            }

            for (var v = 0; v < vertexCount; v++)
            {
                Vertex vertex = mesh.vertexAt(v);
                Debug.Assert(vertex != null);

                // Initial solution.
                x[2 * v + 0] = vertex.tex.x;
                x[2 * v + 1] = vertex.tex.y;
            }

            // Fill A:
            var faceCount = mesh.faceLength;
            for (int f = 0, t = 0; f < faceCount; f++)
            {
                var face = mesh.faceAt(f);
                Debug.Assert(face != null);
                Debug.Assert(face.edgeLength == 3);

                Vertex vertex0 = null;

                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    Edge edge = it.current();
                    Debug.Assert(edge != null);

                    if (vertex0 == null)
                    {
                        vertex0 = edge.vertex;
                    } else if (edge.next.vertex != vertex0)
                    {
                        Vertex vertex1 = edge.from();
                        Vertex vertex2 = edge.to();

                        setup_abf_relations(A, t, vertex0, vertex1, vertex2);
                        //setup_conformal_map_relations(A, t, vertex0, vertex1, vertex2);

                        t++;
                    }
                }
            }


            // Solve
            LeastSquaresSolver(A, b, x, lockedParameters, (uint)lockedParameters.Count, 0.000001f);

            // Map x back to texcoords:
            for (var v = 0; v < vertexCount; v++)
            {
                Vertex vertex = mesh.vertexAt(v);
                Debug.Assert(vertex != null);

                vertex.tex = new Vector2(x[2 * v + 0], x[2 * v + 1]);
            }

            return true;
        }


        public static void EigenSolver3_Tridiagonal(float[,] mat/*[3][3]*/, float[] diag, float[] subd)
        {
            // Householder reduction T = Q^t M Q
            //   Input:   
            //     mat, symmetric 3x3 matrix M
            //   Output:  
            //     mat, orthogonal matrix Q
            //     diag, diagonal entries of T
            //     subd, subdiagonal entries of T (T is symmetric)
            const float epsilon = 1e-08f;

            float a = mat[0,0];
            float b = mat[0,1];
            float c = mat[0,2];
            float d = mat[1,1];
            float e = mat[1,2];
            float f = mat[2,2];

            diag[0] = a;
            subd[2] = 0.0f;
            if (Mathf.Abs(c) >= epsilon)
            {
                var ell = Mathf.Sqrt(b*b+c*c);
                b /= ell;
                c /= ell;
                var q = 2*b*e+c*(f-d);
                diag[1] = d+c*q;
                diag[2] = f-c*q;
                subd[0] = ell;
                subd[1] = e-b*q;
                mat[0,0] = 1; mat[0,1] = 0; mat[0,2] = 0;
                mat[1,0] = 0; mat[1,1] = b; mat[1,2] = c;
                mat[2,0] = 0; mat[2,1] = c; mat[2,2] = -b;
            }
            else
            {
                diag[1] = d;
                diag[2] = f;
                subd[0] = b;
                subd[1] = e;
                mat[0,0] = 1; mat[0,1] = 0; mat[0,2] = 0;
                mat[1,0] = 0; mat[1,1] = 1; mat[1,2] = 0;
                mat[2,0] = 0; mat[2,1] = 0; mat[2,2] = 1;
            }
        }

        public static bool EigenSolver3_QLAlgorithm(float[,] mat/*[3][3]*/, float[] diag, float[] subd)
        {
            // QL iteration with implicit shifting to reduce matrix from tridiagonal
            // to diagonal
            const int maxiter = 32;

            for (int ell = 0; ell < 3; ell++)
            {
                int iter;
                for (iter = 0; iter < maxiter; iter++)
                {
                    int m;
                    for (m = ell; m <= 1; m++)
                    {
                        float dd = Mathf.Abs(diag[m]) + Mathf.Abs(diag[m+1]);
                        if (Mathf.Abs(subd[m]) + dd == dd )
                            break;
                    }
                    if ( m == ell )
                        break;

                    float g = (diag[ell+1]-diag[ell])/(2*subd[ell]);
                    float r = Mathf.Sqrt(g*g+1);
                    if ( g < 0 )
                        g = diag[m]-diag[ell]+subd[ell]/(g-r);
                    else
                        g = diag[m]-diag[ell]+subd[ell]/(g+r);
                    float s = 1, c = 1, p = 0;
                    for (int i = m-1; i >= ell; i--)
                    {
                        float f = s*subd[i], b = c*subd[i];
                        if (Mathf.Abs(f) >= Mathf.Abs(g) )
                        {
                            c = g/f;
                            r = Mathf.Sqrt(c*c+1);
                            subd[i+1] = f*r;
                            c *= (s = 1/r);
                        }
                        else
                        {
                            s = f/g;
                            r = Mathf.Sqrt(s*s+1);
                            subd[i+1] = g*r;
                            s *= (c = 1/r);
                        }
                        g = diag[i+1]-p;
                        r = (diag[i]-g)*s+2*b*c;
                        p = s*r;
                        diag[i+1] = g+p;
                        g = c*r-b;

                        for (int k = 0; k < 3; k++)
                        {
                            f = mat[k,i+1];
                            mat[k,i+1] = s*mat[k,i]+c*f;
                            mat[k,i] = c*mat[k,i]-s*f;
                        }
                    }
                    diag[ell] -= p;
                    subd[ell] = g;
                    subd[m] = 0;
                }

                if ( iter == maxiter )
                    // should not get here under normal circumstances
                    return false;
            }

            return true;
        }

        public static bool eigenSolveSymmetric3(float[] matrix/*[6]*/, float[] eigenValues/*[3]*/, Vector3[] eigenVectors/*[3]*/)
        {
            Debug.Assert(matrix != null && eigenValues != null && eigenVectors != null);

            float[] subd = new float[3];
            float[] diag = new float[3];
            float[,] work = new float[3,3];

            work[0,0] = matrix[0];
            work[0,1] = work[1,0] = matrix[1];
            work[0,2] = work[2,0] = matrix[2];
            work[1,1] = matrix[3];
            work[1,2] = work[2,1] = matrix[4];
            work[2,2] = matrix[5];

            EigenSolver3_Tridiagonal(work, diag, subd);
            if (!EigenSolver3_QLAlgorithm(work, diag, subd))
            {
                for (int i = 0; i < 3; i++) {
                    eigenValues[i] = 0;
                    eigenVectors[i] = Vector3.zero;
                }
                return false;
            }

            for (int i = 0; i < 3; i++) {
                eigenValues[i] = (float)diag[i];
            }

            // eigenvectors are the columns; make them the rows :

            for (int i=0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    eigenVectors[j][i] = (float) work[i,j];
                }
            }

            // shuffle to sort by singular value :
            if (eigenValues[2] > eigenValues[0] && eigenValues[2] > eigenValues[1])
            {
                AtlasPacker.swap(ref eigenValues[0], ref eigenValues[2]);
                AtlasPacker.swap(ref eigenVectors[0], ref eigenVectors[2]);
            }
            if (eigenValues[1] > eigenValues[0])
            {
                AtlasPacker.swap(ref eigenValues[0], ref eigenValues[1]);
                AtlasPacker.swap(ref eigenVectors[0], ref eigenVectors[1]);
            }
            if (eigenValues[2] > eigenValues[1])
            {
                AtlasPacker.swap(ref eigenValues[1], ref eigenValues[2]);
                AtlasPacker.swap(ref eigenVectors[1], ref eigenVectors[2]);
            }

            Debug.Assert(eigenValues[0] >= eigenValues[1] && eigenValues[0] >= eigenValues[2]);
            Debug.Assert(eigenValues[1] >= eigenValues[2]);

            return true;
        }

        const float NV_EPSILON = 0.0001f;
        public static bool isPlanar(int n, Vector3[] points, float epsilon=NV_EPSILON)
        {
            // compute the centroid and covariance
            var matrix = new float[6];
            computeCovariance(n, points, matrix);

            var eigenValues = new float[3];
            var eigenVectors = new Vector3[3];
            if (!eigenSolveSymmetric3(matrix, eigenValues, eigenVectors)) {
                return false;
            }

            return eigenValues[2] < epsilon;
        }

        public static Vector3 computeCentroid(int n, Vector3[] points)
        {
            var centroid = Vector3.zero;

            for (int i = 0; i < n; i++)
                centroid += points[i];
            centroid /= (float)(n);

            return centroid;
        }

        public static Vector3 computeCovariance(int n, Vector3[] points, float[] covariance)
        {
            // compute the centroid
            Vector3 centroid = computeCentroid(n, points);

            // compute covariance matrix
            for (var i = 0; i < 6; i++)
            {
                covariance[i] = 0.0f;
            }

            for (var i = 0; i < n; i++)
            {
                Vector3 v = points[i] - centroid;

                covariance[0] += v.x * v.x;
                covariance[1] += v.x * v.y;
                covariance[2] += v.x * v.z;
                covariance[3] += v.y * v.y;
                covariance[4] += v.y * v.z;
                covariance[5] += v.z * v.z;
            }

            return centroid;
        }

        bool computeOrthogonalProjectionMap(ChartMesh mesh)
        {
            Vector3[] axis = new Vector3[2];

            var vertexCount = mesh.vertexLength;
            var points = new Vector3[vertexCount];

            for (var i = 0; i < vertexCount; i++)
            {
                points[i] = mesh.vertexAt(i).pos;
            }

            // Avoid redundant computations.
            float[] matrix = new float[6];
            computeCovariance(vertexCount, points, matrix);

            if (matrix[0] == 0 && matrix[3] == 0 && matrix[5] == 0) {
                return false;
            }

            float[] eigenValues = new float[3];
            Vector3[] eigenVectors = new Vector3[3];
            if (!eigenSolveSymmetric3(matrix, eigenValues, eigenVectors)) {
                return false;
            }

            axis[0] = eigenVectors[0].normalized;
            axis[1] = eigenVectors[1].normalized;

            // Project vertices to plane.
            for (var it = mesh.vertices(); !it.isDone(); it.advance())
            {
                var vertex = it.current();
                vertex.tex.x = Vector3.Dot(axis[0], vertex.pos);
                vertex.tex.y = Vector3.Dot(axis[1], vertex.pos);
            }

            return true;
        }

        public void ParameterizeCharts(bool preserve_uvs, bool pinned_boundary)
        {
            var globalParameterizationQuality = new ParameterizationQuality();

            // Parameterize the charts.
            var diskCount = 0;
            var chartCount = m_chartArray.Count;
            for (var i = 0; i < chartCount; i++)
            {
                Chart chart = m_chartArray[i];

                bool isValid = false;

                if (chart.isVertexMapped()) {
                    continue;
                }

                if (!preserve_uvs)
                {
                    if (chart.isDisk())
                    {
                        diskCount++;

                        if (chart.faceLength == 1) {
                            computeSingleFaceMap(chart.unifiedMesh());
                        }
                        else {
                            if (pinned_boundary) {
                                computeLeastSquaresConformalMap(chart.unifiedMesh(), true);
                            }
                            else {
                                computeOrthogonalProjectionMap(chart.unifiedMesh());
                                var orthogonalQuality = new ParameterizationQuality(chart.unifiedMesh());

                                computeLeastSquaresConformalMap(chart.unifiedMesh(), pinned_boundary);
                                var lscmQuality = new ParameterizationQuality(chart.unifiedMesh());

                                // If the orthogonal projection produces better results, just use that.
                                // @@ It may be dangerous to do this, because isValid() does not detect self-overlaps.
                                // @@ Another problem is that with very thin patches with nearly zero parametric area, the results of our metric are not accurate.
                                if (orthogonalQuality.isValid() && orthogonalQuality.rmsStretchMetric() < lscmQuality.rmsStretchMetric()) {
                                    computeOrthogonalProjectionMap(chart.unifiedMesh());
                                    //chartParameterizationQuality = orthogonalQuality;
                                }
                                else {
                                    //chartParameterizationQuality = lscmQuality;
                                }
                            }

                            // If conformal map failed, 

                            // @@ Experiment with other parameterization methods.
                            //computeCircularBoundaryMap(chart.unifiedMesh());
                            //computeConformalMap(chart.unifiedMesh());
                            //computeNaturalConformalMap(chart.unifiedMesh());
                            //computeGuidanceGradientMap(chart.unifiedMesh());
                        }
                    }
                }

                var chartParameterizationQuality = new ParameterizationQuality(chart.unifiedMesh());

                isValid = chartParameterizationQuality.isValid();

                if (!isValid)
                {
                    Debug.Log("*** Invalid parameterization.\n");
                }

                // @@ Check that parameterization quality is above a certain threshold.

                // @@ Detect boundary self-intersections.

                globalParameterizationQuality += chartParameterizationQuality;

                if (!isValid)
                {
                    //nvDebugBreak();
                    // @@ Run the builder again, but only on this chart.
                    //AtlasBuilder builder(chart.chartMesh());
                }

                // Transfer parameterization from unified mesh to chart mesh.
                chart.transferParameterization();
            }

            Debug.Log($"  Parameterized {diskCount}/{chartCount} charts.");
            Debug.Log($"  RMS stretch metric: {globalParameterizationQuality.rmsStretchMetric()}");
            Debug.Log($"  MAX stretch metric: {globalParameterizationQuality.maxStretchMetric()}");
            Debug.Log($"  RMS conformal metric: {globalParameterizationQuality.rmsConformalMetric()}");
            Debug.Log($"  RMS authalic metric: {globalParameterizationQuality.maxAuthalicMetric()}");
        }

        ChartMesh m_mesh;

        List<Chart> m_chartArray = new List<Chart>();
        
        uint[] m_chartVertexCountPrefixSum = new uint[0];
        int m_totalVertexCount;

        uint[] m_faceChart = new uint[0]; // the chart of every face of the input mesh.
        uint[] m_faceIndex = new uint[0]; // the index within the chart for every face of the input mesh.
    };

    /// An atlas is a set of charts.
    class Atlas
    {
        public int meshLength { get { return m_meshChartsArray.Count; } }
        public MeshCharts meshAt(int i) { return m_meshChartsArray[i]; }

        public int chartLength
        {
            get
            {
                var count = 0;
                for (var c = 0; c < m_meshChartsArray.Count; c++) {
                    count += m_meshChartsArray[c].chartLength;
                }
                return count;
            }
        }
        
        public Chart chartAt(int i)
        {
            for (var c = 0; c < m_meshChartsArray.Count; c++)
            {
                var count = m_meshChartsArray[c].chartLength;

                if (i < count)
                {
                    return m_meshChartsArray[c].chartAt(i);
                }

                i -= count;
            }

            return null;
        }
        

        
        public void ExtractCharts(ChartMesh mesh, bool ensure_disk_charts)
        {
            var meshCharts = new MeshCharts(mesh);
            meshCharts.extractCharts(ensure_disk_charts);
            addMeshCharts(meshCharts);
        }

        public void ComputeCharts(ChartMesh mesh, SegmentationSettings settings, List<uint> unchartedMaterialArray)
        {
            var meshCharts = new MeshCharts(mesh);
            meshCharts.computeCharts(settings, unchartedMaterialArray);
            addMeshCharts(meshCharts);
        }

        
        public void ParameterizeCharts(bool preserve_uvs, bool pinned_boundary)
        {
            for (int i=0;i< m_meshChartsArray.Count;i++)
            { 
                m_meshChartsArray[i].ParameterizeCharts(preserve_uvs, pinned_boundary);
            }
        }


        public float PackCharts(int quality, float texelsPerUnit, bool blockAlign, bool conservative)
        {
            var packer = new AtlasPacker(this);
            packer.PackCharts(quality, texelsPerUnit, blockAlign, conservative);
            return packer.computeAtlasUtilization();
        }
        
        // Extract the charts and add to this atlas.
        void addMeshCharts(MeshCharts meshCharts)
        {
            m_meshChartsArray.Add(meshCharts);
        }

        // Add mesh charts and takes ownership.
        //public void addMeshCharts(MeshCharts[] meshCharts);

        //public void extractCharts(ChartMesh mesh, bool ensure_disk_charts);
        //public void computeCharts(ChartMesh mesh, ref SegmentationSettings settings, ref uint[] unchartedMaterialArray);


        // Compute a trivial seamless texture similar to ZBrush.
        //bool computeSeamlessTextureAtlas(bool groupFaces = true, bool scaleTiles = false, uint w = 1024, uint h = 1024);

        //public void parameterizeCharts(bool preserve_uvs, bool pinned_boundary);

        // Pack charts in the smallest possible rectangle.
        //public float packCharts(int quality, float texelArea, bool blockAlign, bool conservative);

        List<MeshCharts> m_meshChartsArray = new List<MeshCharts>();
    };
    
    public class SegmentationSettings
    {
        public SegmentationSettings()
        {
            // Charts have no area or boundary limits right now.
            maxChartArea = float.MaxValue;
            maxBoundaryLength = float.MaxValue;

            proxyFitMetricWeight = 1.0f;
            roundnessMetricWeight = 0.1f;
            straightnessMetricWeight = 0.25f;
            normalSeamMetricWeight = 1.0f;
            textureSeamMetricWeight = 0.1f;
        }

        public float maxChartArea;
        public float maxBoundaryLength;

        public float proxyFitMetricWeight;
        public float roundnessMetricWeight;
        public float straightnessMetricWeight;
        public float normalSeamMetricWeight;
        public float textureSeamMetricWeight;
    };

    // Half edge vertex.
    public class Vertex
    {
        public uint id;

        public Edge edge;
        public Vertex next;
        public Vertex prev;

        public Vector3 pos;
        public Vector3 nor;
        public Vector2 tex;
        public Vector4 col;


        public Vertex(uint id) {
            this.id = id; edge = null; pos = Vector3.zero; nor = Vector3.zero; tex = Vector2.zero; col = Vector2.zero;
            next = this;
            prev = this;
        }

        // Set first edge of all colocals.
        public void setEdge(Edge e)
        {
            for (var it = colocals(); !it.isDone(); it.advance())
            {
                it.current().edge = e;
            }
        }

        public void setPos(Vector3 p)
        {
            for (var it = colocals(); !it.isDone(); it.advance())
            {
                it.current().pos = p;
            }
        }

        public int colocalLength
        {
            get
            {
                var count = 0;
                for (var it = colocals(); !it.isDone(); it.advance()) { ++count; }
                return count;
            }
        }

        public int valence()
        {
            var count = 0;
            for (var it = edges(); !it.isDone(); it.advance()) { ++count; }
            return count;
        }

        public bool isFirstColocal()
        {
            return firstColocal() == this;
        }

        public Vertex firstColocal()
        {
            Vertex vertex = this;
            uint firstId = id;

            for (var it = colocals(); !it.isDone(); it.advance())
            {
                if (it.current().id < firstId)
                {
                    firstId = vertex.id;
                    vertex = it.current();
                }
            }

            return vertex;
        }

        public bool isColocal(Vertex v)
        {
            if (this == v) return true;
            if (pos != v.pos) return false;

            for (var it = colocals(); !it.isDone(); it.advance())
            {
                if (v == it.current())
                {
                    return true;
                }
            }

            return false;
        }
        
        // Iterator that visits the edges around this vertex in counterclockwise order.
        public class EdgeIterator //: public Iterator<Edge *>
        {
            public EdgeIterator(Edge e) { m_end = null; m_current = e; }

            public void advance()
            {
                if (m_end == null) m_end = m_current;
                m_current = m_current.pair.next;
                //m_current = m_current.prev.pair;
            }

            public bool isDone()  { return m_end == m_current; }
            public Edge current() { return m_current; }
            public Vertex vertex() { return m_current.vertex; }

            Edge m_end;
            Edge m_current;
        };

        public EdgeIterator edges() { return new EdgeIterator(edge); }

        // Iterator that visits all the colocal vertices.
        public class VertexIterator //: public Iterator<Edge *>
        {
            public VertexIterator(Vertex v) { m_end = null; m_current = v; }

            public void advance()
            {
                if (m_end == null) m_end = m_current;
                m_current = m_current.next;
            }

            public bool isDone() { return m_end == m_current; }
            public Vertex current() { return m_current; }

            Vertex m_end;
            Vertex m_current;
        };

        public VertexIterator colocals() { return new VertexIterator(this); }


        public void linkColocal(Vertex v)
        {
            next.prev = v;
            v.next = next;
            next = v;
            v.prev = this;
        }
        public void unlinkColocal()
        {
            next.prev = prev;
            prev.next = next;
            next = this;
            prev = this;
        }


        // @@ Note: This only works if linkBoundary has been called.
        public bool isBoundary()
        {
            return (edge != null && edge.face == null);
        }



    };


    /// Face of a half-edge mesh.
    public class Face
    {
        public uint id;
        public ushort group;
        public ushort material;
        public Edge edge;


        public Face(uint id) { unchecked { this.id = id; group = (ushort)~((ushort)0); material = (ushort)~((ushort)0); edge = null; } }

        public float area()
        {
            float area = 0;
            var v0 = edge.from().pos;

            for (var it = edges(edge.next); it.current() != edge.prev; it.advance())
            {
                var e = it.current();

                var v1 = e.vertex.pos;
                var v2 = e.next.vertex.pos;

                area += Vector3.Cross(v1 - v0, v2 - v0).magnitude;
            }

            return area * 0.5f;
        }

        // Note, this is the area scaled by 2!
        public static float triangleArea(Vector2 v0, Vector2 v1)
        {
            return (v0.x * v1.y - v0.y * v1.x); // * 0.5f;
        }

        public static float triangleArea(Vector2 a, Vector2 b, Vector2 c)
        {
            // IC: While it may be appealing to use the following expression:
            //return (c.x * a.y + a.x * b.y + b.x * c.y - b.x * a.y - c.x * b.y - a.x * c.y); // * 0.5f;

            // That's actually a terrible idea. Small triangles far from the origin can end up producing fairly large floating point 
            // numbers and the results becomes very unstable and dependent on the order of the factors.

            // Instead, it's preferable to subtract the vertices first, and multiply the resulting small values together. The result
            // in this case is always much more accurate (as long as the triangle is small) and less dependent of the location of 
            // the triangle.

            //return ((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)); // * 0.5f;
            return triangleArea(a - c, b - c);
        }

        public float parametricArea()
        {
            float area = 0;
            var v0 = edge.from().tex;

            for (var it = edges(edge.next); it.current() != edge.prev; it.advance())
            {
                var e = it.current();

                var v1 = e.vertex.tex;
                var v2 = e.next.vertex.tex;

                area += triangleArea(v0, v1, v2);
            }

            return area * 0.5f;
        }


        const float NV_EPSILON = 0.0001f;
        public static bool isZero(float f, float epsilon = NV_EPSILON)
        {
            return Math.Abs(f) <= epsilon;
        }

        public static Vector3 scale(Vector3 v, float s)
        {
            return new Vector3(v.x * s, v.y * s, v.z * s);
        }

        public static Vector3 normalizeSafe(Vector3 v, Vector3 fallback, float epsilon = NV_EPSILON)
        {
            float l = v.magnitude;
            if (isZero(l, epsilon))
            {
                return fallback;
            }
            return scale(v, 1.0f / l);
        }

        //public float boundaryLength() { throw new NotImplementedException(); }
        public Vector3 normal()
        {
            Vector3 n = Vector3.zero;

            Vertex vertex0 = null;

            for (var it = edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();
                Debug.Assert(edge != null);

                if (vertex0 == null)
                {
                    vertex0 = edge.vertex;
                } else if (edge.next.vertex != vertex0)
                {
                    var vertex1 = edge.from();
                    var vertex2 = edge.to();

                    var p0 = vertex0.pos;
                    var p1 = vertex1.pos;
                    var p2 = vertex2.pos;

                    var v10 = p1 - p0;
                    var v20 = p2 - p0;

                    n += Vector3.Cross(v10, v20);
                }
            }

            return normalizeSafe(n, new Vector3(0, 0, 1), 0.0f);
        }
        public Vector3 centroid()
        {
            Vector3 sum = Vector3.zero;
            uint count = 0;

            for (var it = edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();
                sum += edge.from().pos;
                count++;
            }

            return sum / (float)(count);
        }

        public bool isValid()
        {
            uint count = 0;

            for (var it = edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();
                if (edge.face != this) return false;
                if (!edge.isValid()) return false;
                if (!edge.pair.isValid()) return false;
                count++;
            }

            if (count < 3) return false;

            return true;
        }

        public bool contains(Edge e)
        {
            for (var it = edges(); !it.isDone(); it.advance())
            {
                if (it.current() == e) return true;
            }
            return false;
        }

        public const uint NIL = unchecked ((uint)~0);

        public uint edgeIndex(Edge e)
        {
            int i = 0;
            for (var it = edges(); !it.isDone(); it.advance(), i++)
            {
                if (it.current() == e) return (uint)i;
            }
            return NIL;
        }

        public Edge edgeAt(int idx)
        {
            int i = 0;
            for (var it = edges(); !it.isDone(); it.advance(), i++)
            {
                if (i == idx) return it.current();
            }
            return null;
        }

        public int edgeLength { get 
            {
                var count = 0;
                for (var it= edges(); !it.isDone(); it.advance()) { ++count; }
                return count;
            } }
        public bool isBoundary() 
        {
            for (var it = edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();
                Debug.Assert(edge.pair != null);

                if (edge.pair.face == null)
                {
                    return true;
                }
            }
            return false;
        }
        //public int boundaryLength { get { throw new NotImplementedException(); } }
        

        // The iterator that visits the edges of this face in clockwise order.
        public class EdgeIterator //: public Iterator<Edge *>
        {
            public EdgeIterator(Edge e) { m_end = null; m_current = e; }

            public void advance()
            {
                if (m_end == null) m_end = m_current;
                m_current = m_current.next;
            }

            public bool isDone() { return m_end == m_current; }
            public Edge current() { return m_current; }
            public Vertex vertex()  { return m_current.vertex; }

            Edge m_end;
            Edge m_current;
        };

        public EdgeIterator edges() { return new EdgeIterator(edge); }
        public EdgeIterator edges(Edge e)
        { 
            Debug.Assert(contains(e));
            return new EdgeIterator(e); 
        }
    };


    public class Edge
    {

        public uint id;

        public Edge next;
        public Edge prev;	// This is not strictly half-edge, but makes algorithms easier and faster.
        public Edge pair;
        public Vertex vertex;
        public Face face;


        // Default constructor.
        public Edge(uint id) { this.id = id; next = null; prev = null; pair = null; vertex = null; face = null; }

        // Vertex queries.
        public Vertex from() { return vertex; }

        public Vertex to() { return pair.vertex; }  // This used to be 'next.vertex', but that changed often when the connectivity of the mesh changes.


        // Edge queries.
        public void setNext(Edge e) { next = e; if (e != null) e.prev = this; }
        public void setPrev(Edge e) { prev = e; if (e != null) e.next = this; }

        // @@ Add these helpers:
        //Edge * nextBoundary();
        //Edge * prevBoundary();


        // @@ It would be more simple to only check m_pair == null
        // Face queries.
        public bool isBoundary() { return !(face != null && pair.face != null); }

        // @@ This is not exactly accurate, we should compare the texture coordinates...
        public bool isSeam() { return vertex != pair.next.vertex || next.vertex != pair.vertex; }

        public bool isValid()
        {
            // null face is OK.
            if (next == null || prev == null || pair == null || vertex == null) return false;
            if (next.prev != this) return false;
            if (prev.next != this) return false;
            if (pair.pair != this) return false;
            return true;
        }

        // Geometric queries.
        public Vector3 midPoint()
        {
            return (to().pos + from().pos) * 0.5f;
        }
        
        public float length()
        {
            return (to().pos - from().pos).magnitude;
        }
        public float angle()
        {
            var p = vertex.pos;
            var a = prev.vertex.pos;
            var b = next.vertex.pos;

            var v0 = a - p;
            var v1 = b - p;

            return (float)Math.Acos(Vector3.Dot(v0, v1) / (v0.magnitude * v1.magnitude));
        }

    };


    /// Simple half edge mesh designed for dynamic mesh manipulation.
    public class ChartMesh
    {

        public ChartMesh()
        {
            colocalVertexCount = 0;
            errorCount = 0;
        }

        public ChartMesh(ChartMesh mesh)
        {
            errorCount = 0;

            // Copy mesh vertices.
            var vertexCount = mesh.vertexLength;
            m_vertexArray = new List<Vertex>(vertexCount);

            for (var v = 0; v < vertexCount; v++)
            {
                var vertex = mesh.vertexAt(v);
                Debug.Assert(vertex.id == v);

                m_vertexArray.Add(new Vertex((uint)v)
                {
                    pos = vertex.pos,
                    nor = vertex.nor,
                    tex = vertex.tex
                });
            }

            this.colocalVertexCount = vertexCount;


            // Copy mesh faces.
            var faceCount = mesh.faceLength;

            var indexArray = new List<uint>(3);

            for (var f = 0; f < faceCount; f++)
            {
                var face = mesh.faceAt(f);

                for(var it = face.edges(); !it.isDone(); it.advance()) {
                    var vertex = it.current().from();
                    indexArray.Add(vertex.id);
                }

                addFace(indexArray);
                indexArray.Clear();
            }
        }


        public void clear()
        {
            m_vertexArray.Clear();
            m_edgeArray.Clear();
            m_edgeMap.Clear();
            m_faceArray.Clear();
        }

        public Vertex addVertex(Vector3 pos)
        {
            Debug.LogError(AtlasPacker.isFinite(pos));

            var v = new Vertex((uint)m_vertexArray.Count);
            v.pos = pos;
            m_vertexArray.Add(v);

            return v;
        }

        public void linkColocals()
        {
            //Debug.Log("--- Linking colocals:\n");

            var vertexCount = vertexLength;
            var vertexMap = new Dictionary<Vector3, Vertex>(vertexCount);

            for (var v = 0; v < vertexCount; v++)
            {
                var vertex = vertexAt(v);

                if (vertexMap.TryGetValue(vertex.pos, out Vertex colocal))
                {
                    colocal.linkColocal(vertex);
                } else
                {
                    vertexMap[vertex.pos] = vertex;
                }
            }

            this.colocalVertexCount = vertexMap.Count;

            //Debug.Log("---   %d vertex positions.\n", this.colocalVertexCount);

            // @@ Remove duplicated vertices? or just leave them as colocals?
        }

        public void linkColocalsWithCanonicalMap(List<uint> canonicalMap)
        {
            //Debug.Log("--- Linking colocals:\n");

            var vertexMapSize = 0;
            foreach (var i in canonicalMap) 
            {
                vertexMapSize = Math.Max(vertexMapSize, (int)(canonicalMap[(int)i] + 1));
            }

            var vertexMap = new Vertex[vertexMapSize];

            colocalVertexCount = 0;

            var vertexCount = vertexLength;
            for (var v = 0; v < vertexCount; v++)
            {
                var vertex = vertexAt(v);

                var colocal = vertexMap[canonicalMap[v]];
                if (colocal != null)
                {
                    Debug.Assert(vertex.pos == colocal.pos);
                    colocal.linkColocal(vertex);
                } else
                {
                    vertexMap[canonicalMap[v]] = vertex;
                    colocalVertexCount++;
                }
            }

            //Debug.Log("---   %d vertex positions.\n", this.colocalVertexCount); 
        }
        //void resetColocalLinks();

        public Face addFace()
        {
            var f = new Face((uint)m_faceArray.Count);
            m_faceArray.Add(f);
            return f;
        }

        public Face addFace(uint v0, uint v1, uint v2) {

            var indexArray = new[] { v0, v1, v2 };
            return addFace(indexArray, 0, 3);
        }
        public Face addFace(uint v0, uint v1, uint v2, uint v3)
        {
            var indexArray = new[] { v0, v1, v2, v3 };
            return addFace(indexArray, 0, 4);
        }
        public Face addFace(List<uint> indexArray)
        {
            return addFace(indexArray.ToArray(), 0, (uint)indexArray.Count);
        }
        public Face addFace(uint[] indexArray, uint first, uint num)
        {
            Debug.Assert(first < (uint)indexArray.Length);
            Debug.Assert(num <= (uint)(indexArray.Length - first));
            Debug.Assert(num > 2);

            if (!canAddFace(indexArray, first, num))
            {
                errorCount++;
                return null;
            }

            var f = new Face((uint)m_faceArray.Count);

            var firstEdge = (Edge)null;
            var last = (Edge)null;
            Edge current;

            for (var i = 0; i < num - 1; i++)
            {
                current = addEdge(indexArray[first + i], indexArray[first + i + 1]);
                Debug.Assert(current != null && current.face == null);

                current.face = f;

                if (last != null) last.setNext(current);
                else firstEdge = current;

                last = current;
            }

            current = addEdge(indexArray[first + num - 1], indexArray[first]);
            Debug.Assert(current != null && current.face == null);

            current.face = f;

            last.setNext(current);
            current.setNext(firstEdge);

            f.edge = firstEdge;
            m_faceArray.Add(f);

            return f;
        }

        // These functions disconnect the given element from the mesh and delete it.
        public void disconnect(Edge  edge)
        {
            Debug.Assert(edge != null);

            // Remove from edge list.
            if ((edge.id & 1) == 0)
            {
                Debug.Assert(m_edgeArray[(int)edge.id / 2] == edge);
                m_edgeArray[(int)edge.id / 2] = null;
            }

            // Remove edge from map. @@ Store map key inside edge?
            Debug.Assert(edge.from() != null && edge.to() != null);
            bool removed = m_edgeMap.Remove(new Key(edge.from().id, edge.to().id));
            Debug.Assert(removed == true);

            // Disconnect from vertex.
            if (edge.vertex != null)
            {
                if (edge.vertex.edge == edge)
                {
                    if (edge.prev != null && edge.prev.pair != null)
                    {
                        edge.vertex.edge = edge.prev.pair;
                    } else if (edge.pair != null && edge.pair.next != null)
                    {
                        edge.vertex.edge = edge.pair.next;
                    } else
                    {
                        edge.vertex.edge = null;
                        // @@ Remove disconnected vertex?
                    }
                }
                //edge.setVertex(null);
            }

            // Disconnect from face.
            if (edge.face != null)
            {
                if (edge.face.edge == edge)
                {
                    if (edge.next != null && edge.next != edge)
                    {
                        edge.face.edge = edge.next;
                    } else if (edge.prev != null && edge.prev != edge)
                    {
                        edge.face.edge = edge.prev;
                    } else
                    {
                        edge.face.edge = null;
                        // @@ Remove disconnected face?
                    }
                }
                //edge.setFace(null);
            }

            // @@ Hack, we don't disconnect from pair, because pair needs us to remove itself from the map.
            // Disconect from pair.
            /*if (edge.pair != null) {
                if (edge.pair.pair == edge) {
                    edge.pair.setPair(null);
                }
                //edge.setPair(null);
            }*/

            // Disconnect from previous.
            if (edge.prev != null)
            {
                if (edge.prev.next == edge)
                {
                    edge.prev.setNext(null);
                }
                //edge.setPrev(null);
            }

            // Disconnect from next.
            if (edge.next != null)
            {
                if (edge.next.prev == edge)
                {
                    edge.next.setPrev(null);
                }
                //edge.setNext(null);
            }
        }

        //public void disconnectPair(Edge  edge) { throw new NotImplementedException(); }
        //public void disconnect(Vertex  vertex) { throw new NotImplementedException(); }
        //public void disconnect(Face  face) { throw new NotImplementedException(); }

        public void remove(Edge  edge)
        {
            Debug.Assert(edge != null);

            disconnect(edge);
        }
        public void remove(Vertex vertex)
        {
            Debug.Assert(vertex != null);

            // Remove from vertex list.
            m_vertexArray[(int)vertex.id] = null;

            // Disconnect from colocals.
            vertex.unlinkColocal();

            // Disconnect from edges.
            if (vertex.edge != null)
            {
                // @@ Removing a connected vertex is asking for trouble...
                if (vertex.edge.vertex == vertex)
                {
                    // @@ Connect edge to a colocal?
                    vertex.edge.vertex = null;
                }

                vertex.setEdge(null);
            }

            //delete vertex;
        }
            
        public void remove(Face  face)
        {
            Debug.Assert(face != null);

            // Remove from face list.
            m_faceArray[(int)face.id] = null;

            // Disconnect from edges.
            if (face.edge != null)
            {
                Debug.Assert(face.edge.face == face);

                face.edge.face = null;

                face.edge = null;
            }

            //delete face;
        }

        // Remove holes from arrays and reassign indices.
        public void compactEdges()
        {
            var edgeCount = m_edgeArray.Count;

            var c = 0;
            for (var i = 0; i < edgeCount; i++)
            {
                if (m_edgeArray[i] != null)
                {
                    if (i != c)
                    {
                        m_edgeArray[c] = m_edgeArray[i];
                        m_edgeArray[c].id = (uint)(2 * c);
                        if (m_edgeArray[c].pair != null)
                        {
                            m_edgeArray[c].pair.id = (uint)(2 * c + 1);
                        }
                    }
                    c++;
                }
            }

            if (c < m_edgeArray.Count)
                m_edgeArray.RemoveRange(c, m_edgeArray.Count - c);
        }

        public void compactVertices()
        {
            var vertexCount = m_vertexArray.Count;

            var c = 0;
            for (var i = 0; i < vertexCount; i++)
            {
                if (m_vertexArray[i] != null)
                {
                    if (i != c)
                    {
                        m_vertexArray[c] = m_vertexArray[i];
                        m_vertexArray[c].id = (uint)c;
                    }
                    c++;
                }
            }

            if (c < m_vertexArray.Count)
                m_vertexArray.RemoveRange(c, m_vertexArray.Count - c);

            // @@ Generate xref array for external attributes.
        }

        public void compactFaces()
        {
            var faceCount = m_faceArray.Count;

            var c = 0;
            for (var i = 0; i < faceCount; i++)
            {
                if (m_faceArray[i] != null)
                {
                    if (i != c)
                    {
                        m_faceArray[c] = m_faceArray[i];
                        m_faceArray[c].id = (uint)c;
                    }
                    c++;
                }
            }

            if (c < m_faceArray.Count)
                m_faceArray.RemoveRange(c, m_faceArray.Count - c);
        }

        public void triangulate()
        {
            bool all_triangles = true;

            var faceCount = m_faceArray.Count;
            for (var f = 0; f < faceCount; f++)
            {
                var face = m_faceArray[f];
                if (face.edgeLength != 3)
                {
                    all_triangles = false;
                    break;
                }
            }

            if (all_triangles)
            {
                return;
            }


            // Do not touch vertices, but rebuild edges and faces.
            List<Edge> edgeArray = new List<Edge>();
            List<Face> faceArray = new List<Face>();

            AtlasPacker.swap(ref edgeArray, ref m_edgeArray);
            AtlasPacker.swap(ref faceArray, ref m_faceArray);
            m_edgeMap.Clear();

            for (var f = 0; f < faceCount; f++)
            {
                var face = faceArray[f];

                // Trivial fan-like triangulation.
                var v0 = face.edge.vertex.id;
                uint v2;
                uint v1 = uint.MaxValue;

                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    var edge = it.current();
                    v2 = edge.to().id;
                    if (v2 == v0) break;
                    if (v1 != uint.MaxValue) addFace(v0, v1, v2);
                    v1 = v2;
                }
            }

            Debug.Assert(m_faceArray.Count > faceCount); // triangle count > face count

            linkBoundary();
        }

        const float NV_EPSILON = 0.0001f;

        static bool isZero(float f, float epsilon = NV_EPSILON)
        {
            return Math.Abs(f) <= epsilon;
        }

        // Robust floating point comparisons:
        // http://realtimecollisiondetection.net/blog/?p=89
        static bool equal(float f0, float f1, float epsilon = NV_EPSILON)
        {
            //return fabs(f0-f1) <= epsilon;
            return Math.Abs(f0-f1) <= epsilon * Math.Max(1.0f, Math.Max(Math.Abs(f0), Math.Abs(f1)));
        }
        static bool equal(Vector3 v1, Vector3 v2, float epsilon = NV_EPSILON)
        {
            return equal(v1.x, v2.x, epsilon) && equal(v1.y, v2.y, epsilon) && equal(v1.z, v2.z, epsilon);
        }
    
        static Vector3 lerp(Vector3 v1, Vector3 v2, float t)
        {
            float s = 1.0f - t;
            return new Vector3(v1.x * s + t * v2.x, v1.y * s + t * v2.y, v1.z * s + t * v2.z);
        }

        public bool splitBoundaryEdges()
        {
            var boundaryVertices = new List<Vertex>();

            foreach (var v in m_vertexArray) 
            {
                if (v.isBoundary())
                {
                    boundaryVertices.Add(v);
                }
            }

            //Debug.Log("Fixing T-junctions:\n");

            int splitCount = 0;

            foreach (var vertex in boundaryVertices) 
            {
                var x0 = vertex.pos;

                // Find edges that this vertex overlaps with.
                foreach (var edge in m_edgeArray) 
                {
                    if (edge != null && edge.isBoundary())
                    {

                        if (edge.from() == vertex || edge.to() == vertex)
                        {
                            continue;
                        }

                        Vector3 x1 = edge.from().pos;
                        Vector3 x2 = edge.to().pos;

                        Vector3 v01 = x0 - x1;
                        Vector3 v21 = x2 - x1;

                        float l = v21.magnitude;
                        float d = (Vector3.Cross(v01, v21)).magnitude / l;

                        if (isZero(d))
                        {
                            float t = Vector3.Dot(v01, v21) / (l * l);

                            // @@ Snap x0 to x1 or x2, if too close? No, do vertex snapping elsewhere.
                            /*if (equal(t, 0.0f, 0.01f)) {
                                //vertex.setPos(x1);
                            }
                            else if (equal(t, 1.0f, 0.01f)) {
                                //vertex.setPos(x2);
                            }
                            else*/
                            if (t > 0.0f + NV_EPSILON && t < 1.0f - NV_EPSILON)
                            {
                                Debug.Assert(equal(lerp(x1, x2, t), x0));

                                var splitVertex = splitBoundaryEdge(edge, t, x0);
                                vertex.linkColocal(splitVertex);   // @@ Should we do this here?
                                splitCount++;
                            }
                        }
                    }
                }
            }

            if (splitCount > 0)
            {
                Debug.Log($"--- Split {splitCount} edges to fix T-junctions.");
            }

            Debug.Assert(isValid());

            return splitCount != 0;
        } // Returns true if any split was made.
        
        Vertex splitBoundaryEdge(Edge edge, float t, Vector3 pos) 
        {

            /*
                We want to go from this configuration:
           
                    +   +
                    |   ^
                edge |<->|  pair
                    v   |
                    +   +
      
                To this one:

                    +   +
                    |   ^
                    e0 |<->| p0
                    v   |
                vertex +   + 
                    |   ^
                    e1 |<->| p1
                    v   |
                    +   +

            */


            var pair = edge.pair;

            // Make sure boundaries are linked.
            Debug.Assert(pair != null);

            // Make sure edge is a boundary edge.
            Debug.Assert(pair.face == null);

            // Add new vertex.
            var vertex = addVertex(pos);
            vertex.nor = lerp(edge.from().nor, edge.to().nor, t);
            vertex.tex = lerp(edge.from().tex, edge.to().tex, t);
            vertex.col = lerp(edge.from().col, edge.to().col, t);

            disconnect(edge);
            disconnect(pair);

            // Add edges.
            var e0 = addEdge(edge.from().id, vertex.id);
            var p0 = addEdge(vertex.id, pair.to().id);

            var e1 = addEdge(vertex.id, edge.to().id);
            var p1 = addEdge(pair.from().id, vertex.id);

            // Link edges.
            e0.setNext(e1);
            p1.setNext(p0);

            e0.setPrev(edge.prev);
            e1.setNext(edge.next);

            p1.setPrev(pair.prev);
            p0.setNext(pair.next);

            Debug.Assert(e0.next == e1);
            Debug.Assert(e1.prev == e0);

            Debug.Assert(p1.next == p0);
            Debug.Assert(p0.prev == p1);

            Debug.Assert(p0.pair == e0);
            Debug.Assert(e0.pair == p0);

            Debug.Assert(p1.pair == e1);
            Debug.Assert(e1.pair == p1);

            // Link faces.
            e0.face = edge.face;
            e1.face = edge.face;

            // Link vertices.
            edge.from().setEdge(e0);
            vertex.setEdge(e1);

            return vertex;
        }


        // Sew the boundary that starts at the given edge, returns one edge that still belongs to boundary, or null if boundary closed.
        public Edge  sewBoundary(Edge  startEdge) 
        {
            Debug.Assert(startEdge.face == null);

            // @@ We may want to be more conservative linking colocals in order to preserve the input topology. One way of doing that is by linking colocals only 
            // if the vertices next to them are linked as well. That is, by sewing boundaries after detecting them. If any pair of consecutive edges have their first
            // and last vertex in the same position, then it can be linked.

            var lastBoundarySeen = startEdge;

            Debug.Log("Sewing Boundary:\n");

            int count = 0;
            int sewnCount = 0;

            var edge = startEdge;
            do {
                Debug.Assert(edge.face == null);

                var edge_a = edge;
                var edge_b = edge.prev;

                var pair_a = edge_a.pair;
                var pair_b = edge_b.pair;

                var v0a = edge_a.to();
                var v0b = edge_b.from();
                var v1a = edge_a.from();
                var v1b = edge_b.to();

                Debug.Assert(v1a.isColocal(v1b));

                /*
                v0b +      _+ v0a
                     \     /
                    b \   / a
                       \|/
                    v1b + v1a
                */

                // @@ This should not happen while sewing, but it may be produced somewhere else.
                Debug.Assert(edge_a != edge_b);

                if (v0a.pos == v0b.pos) {

                    // Link vertices.
                    v0a.linkColocal(v0b);
            
                    // Remove edges to be collapsed.
                    disconnect(edge_a);
                    disconnect(edge_b);
                    disconnect(pair_a);
                    disconnect(pair_b);

                    // Link new boundary edges.
                    var prevBoundary = edge_b.prev;
                    var nextBoundary = edge_a.next;
                    if (nextBoundary != null) {
                        Debug.Assert(nextBoundary.face == null);
                        Debug.Assert(prevBoundary.face == null);
                        nextBoundary.setPrev(prevBoundary);
            
                        // Make sure boundary vertex points to boundary edge.
                        v0a.setEdge(nextBoundary); // This updates all colocals.
                    }
                    lastBoundarySeen = prevBoundary;

                    // Creat new edge.
                    var newEdge_a = addEdge(v0a.id, v1a.id);   // pair_a.from().id, pair_a.to().id
                    var newEdge_b = addEdge(v1b.id, v0b.id);

                    newEdge_a.pair = newEdge_b;
                    newEdge_b.pair = newEdge_a;

                    newEdge_a.face = pair_a.face;
                    newEdge_b.face = pair_b.face;

                    newEdge_a.setNext(pair_a.next);
                    newEdge_a.setPrev(pair_a.prev);

                    newEdge_b.setNext(pair_b.next);
                    newEdge_b.setPrev(pair_b.prev);

                    //delete edge_a;
                    //delete edge_b;
                    //delete pair_a;
                    //delete pair_b;

                    edge = nextBoundary;    // If nextBoundary is null we have closed the loop.
                    sewnCount++;
                }
                else {
                    edge = edge.next;
                }
        
                count++;
            } while(edge != null && edge != lastBoundarySeen);

            Debug.Log($" - Sewn {sewnCount} out of {count}.");

            if (lastBoundarySeen != null) {
                Debug.Assert(lastBoundarySeen.face == null);
            }

            return lastBoundarySeen; 
        }


        // Vertices
        public int vertexLength { get { return m_vertexArray.Count; } }
        public Vertex  vertexAt(int i) { return m_vertexArray[i]; }

        //uint colocalVertexLength { get { return m_colocalVertexCount; } }

        // Faces
        public int faceLength { get { return m_faceArray.Count; } }
        public Face  faceAt(int i) { return m_faceArray[i]; }

        // Edges
        public int edgeLength { get { return m_edgeArray.Count; } }
        public Edge  edgeAt(int i) { return m_edgeArray[i]; }

        // @@ Add half-edge iterator.
        
        public class VertexIterator
        {
            public VertexIterator(ChartMesh mesh) { m_mesh = mesh; m_current = 0; }

            public void advance() { m_current++; }
            public bool isDone() { return m_current == m_mesh.vertexLength; }
            public Vertex current() { return m_mesh.vertexAt((int)m_current); }

            ChartMesh m_mesh;
            uint m_current;
        };

        public VertexIterator vertices() { return new VertexIterator(this); }

        public class FaceIterator
        {
            public FaceIterator(ChartMesh mesh) { m_mesh = mesh; m_current = 0; }

            public void advance() { m_current++; }
            public bool isDone() { return m_current == m_mesh.faceLength; }
            public Face current() { return m_mesh.faceAt(m_current); }

            ChartMesh m_mesh;
            int m_current;
        };

        public FaceIterator faces() { return new FaceIterator(this); }


        // Convert to tri mesh.
        // public TriMesh  toTriMesh() { throw new NotImplementedException(); }
        //public QuadTriMesh  toQuadTriMesh() { throw new NotImplementedException(); }

        public bool isValid()
        {
            // Make sure all edges are valid.
            var edgeCount = m_edgeArray.Count;
            for (var e = 0; e < edgeCount; e++)
            {
                var edge = m_edgeArray[e];
                if (edge != null)
                {
                    if (edge.id != 2 * e)
                    {
                        return false;
                    }
                    if (!edge.isValid())
                    {
                        return false;
                    }

                    if (edge.pair.id != 2 * e + 1)
                    {
                        return false;
                    }
                    if (!edge.pair.isValid())
                    {
                        return false;
                    }
                }
            }

            // @@ Make sure all faces are valid.

            // @@ Make sure all vertices are valid.

            return true;
        }

        // Error status:
        public uint errorCount;
        public uint errorIndex0;
        public uint errorIndex1;

        public Edge  addEdge(uint i, uint j)
        {
            Debug.Assert(i != j);

            var edge = findEdge(i, j);

            if (edge != null)
            {
                // Edge may already exist, but its face must not be set.
                Debug.Assert(edge.face == null);

                // Nothing else to do!

            } else
            {
                // Add new edge.

                // Lookup pair.
                var pair = findEdge(j, i);

                if (pair != null)
                {
                    // Create edge with same id.
                    edge = new Edge((uint)(pair.id + 1));

                    // Link edge pairs.
                    edge.pair = pair;
                    pair.pair = edge;

                    // @@ I'm not sure this is necessary!
                    pair.vertex.setEdge(pair);
                } else
                {
                    // Create edge.
                    edge = new Edge((uint)(2 * m_edgeArray.Count));

                    // Add only unpaired edges.
                    m_edgeArray.Add(edge);
                }

                edge.vertex = m_vertexArray[(int)i];
                m_edgeMap[new Key(i, j)] = edge;
            }

            // Face and Next are set by addFace.

            return edge;
        }

        private bool canAddFace(uint[] indexArray, uint first, uint num)
        {
            for (uint j = num - 1, i = 0; i < num; j = i++)
            {
                if (!canAddEdge(indexArray[first + j], indexArray[first + i]))
                {
                    errorIndex0 = indexArray[first + j];
                    errorIndex1 = indexArray[first + i];
                    return false;
                }
            }

            // We also have to make sure the face does not have any duplicate edge!
            for (uint i = 0; i < num; i++)
            {

                var i0 = indexArray[(int)(first + i + 0)];
                var i1 = indexArray[(int)(first + (i + 1) % num)];

                for (uint j = i + 1; j < num; j++)
                {
                    var j0 = indexArray[(int)(first + j + 0)];
                    var j1 = indexArray[(int)(first + (j + 1) % num)];

                    if (i0 == j0 && i1 == j1)
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        private bool canAddEdge(uint i, uint j)
        {
            if (i == j)
            {
                // Skip degenerate edges.
                return false;
            }

            // Same check, but taking into account colocal vertices.
            var v0 = vertexAt((int)i);
            var v1 = vertexAt((int)j);

            for (var it = v0.colocals(); !it.isDone(); it.advance())
            {
                if (it.current() == v1)
                {
                    // Skip degenerate edges.
                    return false;
                }
            }

            // Make sure edge has not been added yet.
            var edge = findEdge(i, j);

            return edge == null || edge.face == null; // We ignore edges that don't have an adjacent face yet, since this face could become the edge's face.
        }

        private Edge  findEdge(uint i, uint j)
        {
            Edge edge = null;

            var v0 = vertexAt((int)i);
            var v1 = vertexAt((int)j);

            // Test all colocal pairs.
            for (var it0 = v0.colocals(); !it0.isDone(); it0.advance())
            {
                for (var it1 = v1.colocals(); !it1.isDone(); it1.advance())
                {
                    var key = new Key(it0.current().id, it1.current().id);

                    if (edge == null)
                    {
                        if (m_edgeMap.TryGetValue(key, out edge))
                            return edge;
                    } else
                    {
                        // Make sure that only one edge is found.
                        Debug.Assert(!m_edgeMap.ContainsKey(key));
                    }
                }
            }

            return edge;
        }


        public void linkBoundary() 
        {
            //Debug.Log("--- Linking boundaries:\n");

            int num = 0;

            // Create boundary edges.
            var edgeCount = edgeLength;
            for (var e = 0; e < edgeCount; e++)
            { 
                var edge = edgeAt(e);
                if (edge != null && edge.pair == null)
                {
                    var pair = new Edge(edge.id + 1);

                    uint i = edge.from().id;
                    uint j = edge.next.from().id;

                    var key = new Key(j, i);
                    //Debug.Assert(!m_edgeMap.get(key));

                    pair.vertex = m_vertexArray[(int)j];
                    m_edgeMap[key] = pair;

                    edge.pair = pair;
                    pair.pair = edge;

                    num++;
                }
            }

            // Link boundary edges.
            for (var e = 0; e < edgeCount; e++)
            {
                var edge = edgeAt(e);
                if (edge != null && edge.pair.face == null)
                {
                    linkBoundaryEdge(edge.pair);
                }
            }

            //Debug.Log("---   %d boundary edges.\n", num);
        }

        private void linkBoundaryEdge(Edge edge)
        {
            Debug.Assert(edge.face == null);

            // Make sure next pointer has not been set. @@ We want to be able to relink boundary edges after mesh changes.
            //Debug.Assert(edge.next() == null);

            var next = edge;
            while (next.pair.face != null)
            {
                // Get pair prev
                var e = next.pair.next;
                while (e.next != next.pair)
                {
                    e = e.next;
                }
                next = e;
            }
            edge.setNext(next.pair);

            // Adjust vertex edge, so that it's the boundary edge. (required for isBoundary())
            if (edge.vertex.edge != edge)
            {
                // Multiple boundaries in the same edge.
                //Debug.Assert( edge.vertex().edge() == null || edge.vertex().edge().face() != null );
                edge.vertex.edge = edge;
            }
        }

        //private void splitBoundaryEdge(Edge  edge, Vertex  vertex) { throw new NotImplementedException(); }

        private List<Vertex> m_vertexArray = new List<Vertex>();
        private List<Edge> m_edgeArray = new List<Edge>();
        private List<Face> m_faceArray = new List<Face>();

        private struct Key 
        {
            public Key(Key k) { p0 = k.p0; p1 = k.p1; }
            public Key(uint v0, uint v1) { p0 = v0; p1 = v1; }
            
            public static bool operator ==(Key a, Key b) { return a.p0 == b.p0 && a.p1 == b.p1; }
            public static bool operator !=(Key a, Key b) { return a.p0 != b.p0 || a.p1 != b.p1; }

            public uint p0;
            public uint p1;

            public override bool Equals(object obj)
            {
                if (!(obj is Key))
                    return false;
                var other = (Key)obj;
                return this == other;
            }

            public override int GetHashCode()
            {
                unchecked { return (int)((p0 * p1) ^ (p0 + p1)); }
            }
        };

        private Dictionary<Key, Edge> m_edgeMap = new Dictionary<Key, Edge>();

        public int colocalVertexCount;

    };
    
    

    // Simple bit array.
    class BitArray
    {
        public BitArray() { m_size = 0; }
        public BitArray(uint sz) {
            resize(sz);
        }

        public uint Length { get { return m_size; } }
        public void clear() { resize(0); }

        public void resize(uint new_size)
        {
            m_size = new_size;
            m_wordArray = new uint[(m_size + 31) >> 5];
        }

        public void resize(uint new_size, bool init)
        {
            //if (new_size == m_size) return;

            uint old_size = m_size;
            var size_mod_32 = (int)(old_size & 31);
            uint last_word_index = ((old_size + 31) >> 5) - 1;
            uint mask = (1u << size_mod_32) - 1;

            uint init_dword;
            if (init) {
                if (size_mod_32 != 0) m_wordArray[last_word_index] |= ~mask;
                init_dword = ~0u;
            }
            else {
                if (size_mod_32 != 0) m_wordArray[last_word_index] &= mask;
                init_dword = 0;
            }

            m_size = new_size;
            var oldSize = m_wordArray.Length;
            Array.Resize(ref m_wordArray, (int)((new_size + 31) >> 5));
            for (int n = oldSize; n < m_size; n++)
                m_wordArray[n] = init_dword;

            // Make sure new bits are initialized correctly.
            /*for (uint i = old_size; i < new_size; i++) {
                Debug.Assert(bitAt(i) == init);
            }*/
        }


        /// Get bit.
        public bool bitAt(uint b) 
        {
            Debug.Assert( b < m_size );
            return (m_wordArray[b >> 5] & (1 << ((int)b & 31))) != 0;
        }

        // It may be useful to pack mulitple bit arrays together interleaving their bits.
        public uint bitsAt(uint idx, uint count) 
        {
            //Debug.Assert(count == 2 || count == 4 || count == 8 || count == 16 || count == 32);
            Debug.Assert(count == 2);   // @@ Hardcoded for two.
            uint b = idx * count;
            Debug.Assert(b < m_size);
            return (m_wordArray[b >> 5] & (0x3u << ((int)b & 31))) >> ((int)b & 31);
        }

        // It would be useful to have a function to set two bits simultaneously.
        /*void setBitsAt(uint idx, uint count, uint bits) const
        {
            //Debug.Assert(count == 2 || count == 4 || count == 8 || count == 16 || count == 32);
            Debug.Assert(count == 2);   // @@ Hardcoded for two.
            uint b = idx * count;
            Debug.Assert(b < m_size);
            return (m_wordArray[b >> 5] & (0x3 << (b & 31))) >> (b & 31);
        }*/



        // Set a bit.
        public void setBitAt(uint idx)
        {
            Debug.Assert(idx < m_size);
            m_wordArray[idx >> 5] |=  (1u << ((int)idx & 31));
        }

        // Clear a bit.
        public void clearBitAt(uint idx)
        {
            Debug.Assert(idx < m_size);
            m_wordArray[idx >> 5] &= ~(1u << ((int)idx & 31));
        }

        // Toggle a bit.
        public void toggleBitAt(uint idx)
        {
            Debug.Assert(idx < m_size);
            m_wordArray[idx >> 5] ^= (1u << ((int)idx & 31));
        }



        // See "Conditionally set or clear bits without branching" at http://graphics.stanford.edu/~seander/bithacks.html
        uint setBits(uint w, uint m, bool b)
        {
            unchecked
            {
                return (w & ~m) | ((uint)(-(b ? 1 : 0)) & (uint)m);
            }
        }


        // Set a bit to the given value. @@ Rename modifyBitAt? 
        public void setBitAt(uint idx, bool b)
        {
            Debug.Assert(idx < m_size);
            m_wordArray[idx >> 5] = setBits(m_wordArray[idx >> 5], 1u << ((int)idx & 31), b);
            Debug.Assert(bitAt(idx) == b);
        }

        public void Add(bool value)
        {
            resize(m_size + 1);
            setBitAt(m_size - 1, value);
        }


        // Clear all the bits.
        public void clearAll()
        {
            Array.Clear(m_wordArray, 0, m_wordArray.Length);
        }

        // Set all the bits.
        public void setAll()
        {
            for (int i = 0; i < m_wordArray.Length; i++)
                m_wordArray[i] = ~0u;
        }

        // Toggle all the bits.
        public void toggleAll()
        {
            var wordCount = m_wordArray.Length;
            for(uint b = 0; b < wordCount; b++) {
                m_wordArray[b] ^= 0xFFFFFFFF;
            }
        }


        // @@ Uh, this could be much faster.
        uint countSetBits(uint x)
        {
            uint count = 0;
            for (; x != 0; x >>= 1)
            {
                count += (x & 1);
            }
            return count;
        }

        // @@ This is even more lame. What was I thinking?
        uint countSetBits(uint x, int bits)
        {
            uint count = 0;
            for (; x != 0 && bits != 0; x >>= 1, bits--)
            {
                count += (x & 1);
            }
            return count;
        }


        // Count the number of bits set.
        public uint countSetBits() 
        {
            var num = m_wordArray.Length;
            if( num == 0 ) {
                return 0;
            }

            uint count = 0;				
            for(uint i = 0; i < num - 1; i++) {
                count += countSetBits(m_wordArray[i]);
            }
            count += countSetBits(m_wordArray[num - 1], (int)(m_size & 31));

            //piDebugCheck(count + countClearBits() == m_size);
            return count;
        }

        // Count the number of bits clear.
        public uint countClearBits() 
        {
            var num = m_wordArray.Length;
            if( num == 0 ) {
                return 0;
            }

            uint count = 0;
            for(uint i = 0; i < num - 1; i++) {
                count += countSetBits(~m_wordArray[i]);
            }
            count += countSetBits(~m_wordArray[num - 1], (int)(m_size & 31));

            //piDebugCheck(count + countSetBits() == m_size);
            return count;
        }

        /*
        public void operator &= (const BitArray & other) 
        {
            if (other.m_size != m_size) {
                resize(other.m_size);
            }

            const uint wordCount = m_wordArray.Count;
            for (uint i = 0; i < wordCount; i++) {
                m_wordArray[i] &= other.m_wordArray[i];
            }
        }

        public void operator |= (const BitArray & other) {
            if (other.m_size != m_size) {
                resize(other.m_size);
            }

            const uint wordCount = m_wordArray.Count;
            for (uint i = 0; i < wordCount; i++) {
                m_wordArray[i] |= other.m_wordArray[i];
            }
        }
        */


        // Number of bits stored.
        uint m_size = 0;

        // Array of bits.
        uint[] m_wordArray;

    };

    /// Bit map. This should probably be called BitImage.
    class BitMap
    {
        public BitMap() { m_width = 0; m_height = 0; }
        public BitMap(int w, int h) { m_width = w; m_height = h; m_bitArray = new BitArray((uint)(w * h)); }

        public int width() { return m_width; }
        public int height() { return m_height; }

        public void resize(int w, int h, bool initValue)
        {
            var tmp = new BitArray((uint)(w* h));

            if (initValue) tmp.setAll();
            else tmp.clearAll();

            // @@ Copying one bit at a time. This could be much faster.
            for (int y = 0; y < m_height; y++)
            {
                for (int x = 0; x < m_width; x++)
                {
                    //tmp.setBitAt(y*w + x, bitAt(x, y));
                    if (bitAt(x, y) != initValue) tmp.toggleBitAt((uint)(y * w + x));
                }
            }

            AtlasPacker.swap(ref m_bitArray, ref tmp);
            m_width = w;
            m_height = h;
        }

        public bool bitAt(int x, int y) 
        {
            Debug.Assert(x < m_width && y < m_height);
            return m_bitArray.bitAt((uint)(y * m_width + x));
        }

        public bool bitAt(uint idx) 
        {
            return m_bitArray.bitAt(idx);
        }

        public void setBitAt(int x, int y)
        {
            Debug.Assert(x < m_width && y < m_height);
            m_bitArray.setBitAt((uint)(y * m_width + x));
        }

        public void setBitAt(uint idx)
        {
            m_bitArray.setBitAt(idx);
        }

        public void clearBitAt(int x, int y)
        {
            Debug.Assert(x < m_width && y < m_height);
            m_bitArray.clearBitAt((uint)(y * m_width + x));
        }

        public void clearBitAt(uint idx)
        {
            m_bitArray.clearBitAt(idx);
        }

        public void clearAll()
        {
            m_bitArray.clearAll();
        }

        public void setAll()
        {
            m_bitArray.setAll();
        }

        public void toggleAll()
        {
            m_bitArray.toggleAll();
        }

        int m_width;
        int m_height;
        BitArray m_bitArray;

    };

    class Thekla
    {

        public enum Atlas_Charter 
        {
            Atlas_Charter_Witness,
            Atlas_Charter_Extract,
            Atlas_Charter_Default = Atlas_Charter_Witness
        };

        public enum Atlas_Mapper 
        {
            Atlas_Mapper_LSCM,
            Atlas_Mapper_Default = Atlas_Mapper_LSCM
        };

        public enum Atlas_Packer 
        {
            Atlas_Packer_Witness,
            Atlas_Packer_Default = Atlas_Packer_Witness
        };

        public enum Atlas_Error
        {
            Atlas_Error_Success,
            Atlas_Error_Invalid_Args,
            Atlas_Error_Invalid_Options,
            Atlas_Error_Invalid_Mesh,
            Atlas_Error_Invalid_Mesh_Non_Manifold,
            Atlas_Error_Not_Implemented,
        };

        public class Atlas_Options 
        {
            public Atlas_Charter charter;
            //struct witness            
            public float proxy_fit_metric_weight;
            public float roundness_metric_weight;
            public float straightness_metric_weight;
            public float normal_seam_metric_weight;
            public float texture_seam_metric_weight;
            public float max_chart_area;
            public float max_boundary_length;


            public Atlas_Mapper mapper;
            //struct mapper_options
            public bool preserve_uvs;
            public bool preserve_boundary;

            public Atlas_Packer packer;
            //struct witness
            public int packing_quality;
            public float texels_per_unit;      // Unit to texel scale. e.g. a 1x1 quad with texelsPerUnit of 32 will take up approximately 32x32 texels in the atlas.
            public bool block_align;           // Align charts to 4x4 blocks. 
            public bool conservative;          // Pack charts with extra padding.
        };


        public class Atlas_Input_Vertex
        {
            public float3   position;
            public float3   normal;
            public float2   uv;
            public int      first_colocal;
        };

        public class Atlas_Input_Face
        {
            public int3 vertex_index;
            public int material_index;
        };

        public class Atlas_Input_Mesh
        {
            public int                  vertex_count;
            public int                  face_count;
            public Atlas_Input_Vertex[] vertex_array;
            public Atlas_Input_Face[]   face_array;
        };

        public struct Atlas_Output_Vertex
        {
            public float2   uv;
            public int      xref;   // Index of input vertex from which this output vertex originated.
        };

        public class Atlas_Output_Mesh 
        {
            public int atlas_width;
            public int atlas_height;
            public int vertex_count;
            public int index_count;
            public Atlas_Output_Vertex[] vertex_array;
            public int[] index_array;
        };

        

        static void input_to_mesh(Atlas_Input_Mesh input, ChartMesh mesh, out Atlas_Error error) 
        {
            var canonicalMap = new List<uint>(input.vertex_count);

            for (int i = 0; i < input.vertex_count; i++) 
            {
                var input_vertex = input.vertex_array[i];
                var pos = input_vertex.position;
                var nor = input_vertex.normal;
                var tex = input_vertex.uv;

                var vertex = mesh.addVertex(new Vector3(pos[0], pos[1], pos[2]));
                vertex.nor = new Vector3(nor[0], nor[1], nor[2]);
                vertex.tex = new Vector2(tex[0], tex[1]);

                canonicalMap.Add((uint)input_vertex.first_colocal);
            }

            mesh.linkColocalsWithCanonicalMap(canonicalMap);


            var face_count = input.face_count;

            int non_manifold_faces = 0;
            for (int i = 0; i < face_count; i++) {
                var input_face = input.face_array[i];

                int v0 = input_face.vertex_index[0];
                int v1 = input_face.vertex_index[1];
                int v2 = input_face.vertex_index[2];

                var face = mesh.addFace((uint)v0, (uint)v1, (uint)v2);
                if (face != null) {
                    face.material = (ushort)input_face.material_index;
                }
                else {
                    non_manifold_faces++;
                }
            }

            mesh.linkBoundary();

            if (non_manifold_faces != 0) 
                error = Atlas_Error.Atlas_Error_Invalid_Mesh_Non_Manifold;
            else
                error = Atlas_Error.Atlas_Error_Success;
        }

        

        public void atlas_set_default_options(ref Atlas_Options options) 
        {
            // These are the default values we use on The Witness.

            options.charter = Atlas_Charter.Atlas_Charter_Default;
            options.proxy_fit_metric_weight = 2.0f;
            options.roundness_metric_weight = 0.01f;
            options.straightness_metric_weight = 6.0f;
            options.normal_seam_metric_weight = 4.0f;
            options.texture_seam_metric_weight = 0.5f;
            options.max_chart_area = float.MaxValue;
            options.max_boundary_length = float.MaxValue;

            options.mapper = Atlas_Mapper.Atlas_Mapper_Default;

            options.packer = Atlas_Packer.Atlas_Packer_Default;
            options.packing_quality = 1;    // Avoid brute force packing, since it can be unusably slow in some situations.
            options.texels_per_unit = 8;
            options.block_align = true;
            options.conservative = false;


            options.preserve_uvs        = true;
            options.preserve_boundary   = true;
        }

        static Atlas_Output_Mesh mesh_atlas_to_output(ChartMesh mesh, Atlas atlas, out Atlas_Error error) 
        {
            var output = new Atlas_Output_Mesh();

            var charts = atlas.meshAt(0);

            // Allocate vertices.
            var vertex_count = charts.vertexLength;
            output.vertex_count = vertex_count;
            output.vertex_array = new Atlas_Output_Vertex[vertex_count];

            int w = 0;
            int h = 0;

            // Output vertices.
            var chart_count = charts.chartLength;
            for (var i = 0; i < chart_count; i++) {
                var chart = charts.chartAt(i);
                var vertexOffset = charts.vertexCountBeforeChartAt((uint)i);

                var chart_vertex_count = chart.vertexLength;
                for (var v = 0; v < chart_vertex_count; v++) 
                {
                    var output_vertex = output.vertex_array[vertexOffset + v]; 

                    var original_vertex = chart.mapChartVertexToOriginalVertex(v);
                    output_vertex.xref = (int)original_vertex;

                    var uv = chart.chartMesh().vertexAt(v).tex;
                    output_vertex.uv[0] = uv.x;
                    output_vertex.uv[1] = uv.y;
                    w = Math.Max(w, Mathf.CeilToInt(uv.x));
                    h = Math.Max(h, Mathf.CeilToInt(uv.y));
                }
            }

            var face_count = mesh.faceLength;
            output.index_count = face_count * 3;
            output.index_array = new int[face_count * 3];

            // Set face indices.
            for (var f = 0; f < face_count; f++) 
            {
                var c = charts.faceChartAt((uint)f);
                var i = charts.faceIndexWithinChartAt((uint)f);
                var vertexOffset = charts.vertexCountBeforeChartAt(c);

                var chart = charts.chartAt((int)c);
                Debug.Assert(chart.faceAt((int)i) == f);

                var face = chart.chartMesh().faceAt((int)i);
                var edge = face.edge;

                output.index_array[3*f+0] = (int)(vertexOffset + edge.vertex.id);
                output.index_array[3*f+1] = (int)(vertexOffset + edge.next.vertex.id);
                output.index_array[3*f+2] = (int)(vertexOffset + edge.next.next.vertex.id);
            }

            error = Atlas_Error.Atlas_Error_Success;
            output.atlas_width = w;
            output.atlas_height = h;

            return output;
        }


        public Atlas_Output_Mesh atlas_generate(Atlas_Input_Mesh input, Atlas_Options options, out Atlas_Error error) 
        {
            error = Atlas_Error.Atlas_Error_Success;

            // Validate args.
            if (input == null || options == null)
            {
                error = Atlas_Error.Atlas_Error_Invalid_Args;
                return null;
            }

            // Validate options.
            if (options.charter != Atlas_Charter.Atlas_Charter_Witness && options.charter != Atlas_Charter.Atlas_Charter_Extract) 
            {
                error = Atlas_Error.Atlas_Error_Invalid_Options;
                return null;
            }
            
            if (options.charter == Atlas_Charter.Atlas_Charter_Witness) {
                // @@ Validate input options!
            }

            if (options.mapper != Atlas_Mapper.Atlas_Mapper_LSCM) 
            {
                error = Atlas_Error.Atlas_Error_Invalid_Options;
                return null;
            }

            if (options.mapper == Atlas_Mapper.Atlas_Mapper_LSCM) 
            {
                // No options.
            }

            if (options.packer != Atlas_Packer.Atlas_Packer_Witness) 
            {
                error = Atlas_Error.Atlas_Error_Invalid_Options;
                return null;
            }

            if (options.packer == Atlas_Packer.Atlas_Packer_Witness) 
            {
                // @@ Validate input options!
            }

            // Validate input mesh.
            for (int i = 0; i < input.face_count; i++) 
            {
                int v0 = input.face_array[i].vertex_index[0];
                int v1 = input.face_array[i].vertex_index[1];
                int v2 = input.face_array[i].vertex_index[2];

                if (v0 < 0 || v0 >= input.vertex_count || 
                    v1 < 0 || v1 >= input.vertex_count || 
                    v2 < 0 || v2 >= input.vertex_count)
                {
                    error = Atlas_Error.Atlas_Error_Invalid_Mesh;
                    return null;
                }
            }


            // Build half edge mesh.
            ChartMesh mesh = new ChartMesh();

            input_to_mesh(input, mesh, out error);

            if (error != Atlas_Error.Atlas_Error_Success) 
                return null;

            Atlas atlas = new Atlas(); 

            // Charter.
            if (options.charter == Atlas_Charter.Atlas_Charter_Extract) 
            {
                // Let's only care about the chart topology if we have to run the LSCM mapper from scratch
                bool ensure_disk_charts = options.preserve_uvs == false || options.preserve_boundary == false;
                atlas.ExtractCharts(mesh, ensure_disk_charts);
            }
            else if (options.charter == Atlas_Charter.Atlas_Charter_Witness) 
            {
                var segmentation_settings = new SegmentationSettings();
                segmentation_settings.proxyFitMetricWeight      = options.proxy_fit_metric_weight;
                segmentation_settings.roundnessMetricWeight     = options.roundness_metric_weight;
                segmentation_settings.straightnessMetricWeight  = options.straightness_metric_weight;
                segmentation_settings.normalSeamMetricWeight    = options.normal_seam_metric_weight;
                segmentation_settings.textureSeamMetricWeight   = options.texture_seam_metric_weight;
                segmentation_settings.maxChartArea              = options.max_chart_area;
                segmentation_settings.maxBoundaryLength         = options.max_boundary_length;

                List<uint> uncharted_materials = new List<uint>();
                atlas.ComputeCharts(mesh, segmentation_settings, uncharted_materials);
            }

            // Mapper.

            if (options.mapper == Atlas_Mapper.Atlas_Mapper_LSCM) 
            {
                atlas.ParameterizeCharts(options.preserve_uvs, options.preserve_boundary);
            }

            // Packer.
            if (options.packer == Atlas_Packer.Atlas_Packer_Witness) 
            {
                int packing_quality     = options.packing_quality;
                float texels_per_unit   = options.texels_per_unit;
                bool block_align        = options.block_align;
                bool conservative       = options.conservative;

                /*float utilization =*/ atlas.PackCharts(packing_quality, texels_per_unit, block_align, conservative);
            }


            // Build output mesh.
            return mesh_atlas_to_output(mesh, atlas, out error);
        }

    }

    class AtlasPacker
    {

        public AtlasPacker(Atlas atlas) 
        {
            m_atlas = atlas;
            m_bitmap = new BitMap(256, 256);
            m_width = 0;
            m_height = 0;

            //m_debug_bitmap.allocate(256, 256);
            //m_debug_bitmap.fill(Color32(0, 0, 0, 0));
        }

        Atlas m_atlas = new Atlas();
        RadixSort m_radix = new RadixSort();
        BitMap m_bitmap = new BitMap();
        int m_width;
        int m_height;

        const float NV_EPSILON = 0.0001f;
        const float NV_NORMAL_EPSILON = 0.001f;

        // Robust floating point comparisons:
        // http://realtimecollisiondetection.net/blog/?p=89
        public static bool equal(float f0, float f1, float epsilon = NV_EPSILON)
        {
            //return fabs(f0-f1) <= epsilon;
            return Math.Abs(f0 - f1) <= epsilon * Math.Max(1.0f, Math.Max(Math.Abs(f0), Math.Abs(f1)));
        }

        public static bool equal(Vector2 v1, Vector2 v2, float epsilon = NV_EPSILON)
        {
            return equal(v1.x, v2.x, epsilon) && equal(v1.y, v2.y, epsilon);
        }

        public static bool isFinite(float value)
        {
            return !float.IsInfinity(value);
        }

        public static bool isFinite(Vector2 value)
        {
            return !float.IsInfinity(value.x) &&
                   !float.IsInfinity(value.y);
        }

        public static int align(int x, int a)
        {
            //return a * ((x + a - 1) / a);
            //return (x + a - 1) & -a;
            return (x + a - 1) & ~(a - 1);
        }

        public static bool isAligned(int x, int a)
        {
            return (x & (a - 1)) == 0;
        }

        /// Swap two values.
        public static void swap<T>(ref T a, ref T b)
        {
            T temp = a;
            a = b;
            b = temp;
        }

        /** Return the next power of two. 
        * @see http://graphics.stanford.edu/~seander/bithacks.html
        * @warning Behaviour for 0 is undefined.
        * @note isPowerOfTwo(x) == true -> nextPowerOfTwo(x) == x
        * @note nextPowerOfTwo(x) = 2 << log2(x-1)
        */
        public static uint nextPowerOfTwo(uint x)
        {
            Debug.Assert(x != 0);
#if true	// On modern CPUs this is supposed to be as fast as using the bsr instruction.
            x--;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
#else
            uint p = 1;
            while (x > p)
            {
                p += p;
            }
            return p;
#endif
        }

        public static long nextPowerOfTwo(long x)
        {
            Debug.Assert(x != 0);
            uint p = 1;
            while (x > p)
            {
                p += p;
            }
            return p;
        }

        public static float square(float f) { return f * f; }
        public static int square(int i) { return i * i; }

        public static float cube(float f) { return f * f * f; }
        public static int cube(int i) { return i * i * i; }


        public static Vector2 max(Vector2 a, Vector2 b)
        {
            return new Vector2(Math.Max(a.x, b.x), Math.Max(a.y, b.y));
        }

        // Note, this is the area scaled by 2!
        public static float triangleArea(Vector2 v0, Vector2 v1)
        {
            return (v0.x * v1.y - v0.y * v1.x); // * 0.5f;
        }

        public static float triangleArea(Vector2 a, Vector2 b, Vector2 c)
        {
            // IC: While it may be appealing to use the following expression:
            //return (c.x * a.y + a.x * b.y + b.x * c.y - b.x * a.y - c.x * b.y - a.x * c.y); // * 0.5f;

            // That's actually a terrible idea. Small triangles far from the origin can end up producing fairly large floating point 
            // numbers and the results becomes very unstable and dependent on the order of the factors.

            // Instead, it's preferable to subtract the vertices first, and multiply the resulting small values together. The result
            // in this case is always much more accurate (as long as the triangle is small) and less dependent of the location of 
            // the triangle.

            //return ((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)); // * 0.5f;
            return triangleArea(a - c, b - c);
        }


        // Compute the convex hull using Graham Scan.
        static void convexHull(List<Vector2> input, List<Vector2> output, float epsilon/*=0*/)
        {
            var inputCount = input.Count;

            var coords = new float[inputCount];

            for (int i = 0; i < inputCount; i++)
            {
                coords[i] = input[i].x;
            }

            RadixSort radix = RadixSort.sort(coords);

            var ranks = radix.ranks();

            var top = new List<Vector2>(inputCount);
            var bottom = new List<Vector2>(inputCount);

            var P = input[ranks[0]];
            var Q = input[ranks[inputCount - 1]];

            var topy = Math.Max(P.y, Q.y);
            var boty = Math.Min(P.y, Q.y);

            for (var i = 0; i < inputCount; i++)
            {
                Vector2 p = input[ranks[i]];
                if (p.y >= boty) top.Add(p);
            }

            for (var i = 0; i < inputCount; i++)
            {
                Vector2 p = input[ranks[inputCount - 1 - i]];
                if (p.y <= topy) bottom.Add(p);
            }

            // Filter top list.
            output.Clear();
            output.Add(top[0]);
            output.Add(top[1]);

            for (int i = 2; i < top.Count;)
            {
                Vector2 a = output[output.Count - 2];
                Vector2 b = output[output.Count - 1];
                Vector2 c = top[i];

                float area = triangleArea(a, b, c); // * 0.5

                if (area >= -epsilon)
                {
                    output.RemoveAt(output.Count - 1);
                }

                if (area < -epsilon || output.Count == 1)
                {
                    output.Add(c);
                    i++;
                }
            }

            var top_count = output.Count;
            output.Add(bottom[1]);

            // Filter bottom list.
            for (int i = 2; i < bottom.Count;)
            {
                Vector2 a = output[output.Count - 2];
                Vector2 b = output[output.Count - 1];
                Vector2 c = bottom[i];

                float area = triangleArea(a, b, c); // * 0.5

                if (area >= -epsilon)
                {
                    output.RemoveAt(output.Count - 1);
                }

                if (area < -epsilon || output.Count == top_count)
                {
                    output.Add(c);
                    i++;
                }
            }

            // Remove duplicate element.
            Debug.Assert(output.First() == output.Last());
            output.RemoveAt(output.Count - 1);
        }

        // This should compute convex hull and use rotating calipers to find the best box. Currently it uses a brute force method.
        static void computeBoundingBox(Chart chart, out Vector2 majorAxis, out Vector2 minorAxis, out Vector2 minCorner, out Vector2 maxCorner)
        {
            // Compute list of boundary points.
            var points = new List<Vector2>(16);

            ChartMesh mesh = chart.chartMesh();
            var vertexCount = mesh.vertexLength;

            for (var i = 0; i < vertexCount; i++)
            {
                Vertex vertex = mesh.vertexAt(i);
                if (vertex.isBoundary())
                {
                    points.Add(vertex.tex);
                }
            }


            var hull = new List<Vector2>();

            convexHull(points, hull, 0.00001f);

            // @@ Ideally I should use rotating calipers to find the best box. Using brute force for now.

            float best_area = float.MaxValue;
            Vector2 best_min = Vector2.zero;
            Vector2 best_max = Vector2.zero;
            Vector2 best_axis = Vector2.zero;

            int hullCount = hull.Count;
            for (int i = 0, j = hullCount - 1; i < hullCount; j = i, i++)
            {

                if (equal(hull[i], hull[j]))
                {
                    continue;
                }

                Vector2 axis = (hull[i] - hull[j]).normalized;
                Debug.Assert(isFinite(axis));

                // Compute bounding box.
                Vector2 box_min = new Vector2(float.MaxValue, float.MaxValue);
                Vector2 box_max = new Vector2(-float.MaxValue, -float.MaxValue);

                for (int v = 0; v < hullCount; v++)
                {

                    Vector2 point = hull[v];

                    float x = Vector2.Dot(axis, point);
                    if (x < box_min.x) box_min.x = x;
                    if (x > box_max.x) box_max.x = x;

                    float y = Vector2.Dot(new Vector2(-axis.y, axis.x), point);
                    if (y < box_min.y) box_min.y = y;
                    if (y > box_max.y) box_max.y = y;
                }

                // Compute box area.
                float area = (box_max.x - box_min.x) * (box_max.y - box_min.y);

                if (area < best_area)
                {
                    best_area = area;
                    best_min = box_min;
                    best_max = box_max;
                    best_axis = axis;
                }
            }


            // Consider all points, not only boundary points, in case the input chart is malformed.
            for (var i = 0; i < vertexCount; i++)
            {
                Vertex vertex = mesh.vertexAt(i);
                Vector2 point = vertex.tex;

                float x = Vector2.Dot(best_axis, point);
                if (x < best_min.x) best_min.x = x;
                if (x > best_max.x) best_max.x = x;

                float y = Vector2.Dot(new Vector2(-best_axis.y, best_axis.x), point);
                if (y < best_min.y) best_min.y = y;
                if (y > best_max.y) best_max.y = y;
            }

            majorAxis = best_axis;
            minorAxis = new Vector2(-best_axis.y, best_axis.x);
            minCorner = best_min;
            maxCorner = best_max;
        }

        
        public float computeAtlasUtilization()  
        {
            var w = m_width;
            var h = m_height;
            Debug.Assert(w <= m_bitmap.width());
            Debug.Assert(h <= m_bitmap.height());

            var count = 0;
            for (var y = 0; y < h; y++) {
                for (var x = 0; x < w; x++) {
                    count += m_bitmap.bitAt(x, y) ? 1 : 0;
                }
            }

            return (float)(count) / (w * h);
        }

        public void PackCharts(int quality, float texelsPerUnit, bool blockAligned, bool conservative)
        {
            Debug.Assert(texelsPerUnit > 0.0f);

            var chartCount = m_atlas.chartLength;
            if (chartCount == 0) return;

            var chartOrderArray = new float[chartCount];
            var chartExtents = new Vector2[chartCount];

            float meshArea = 0;
            for (var c = 0; c < chartCount; c++)
            {
                Chart chart = m_atlas.chartAt(c);

                if (!chart.isVertexMapped() && !chart.isDisk())
                {
                    chartOrderArray[c] = 0;

                    // Skip non-disks.
                    continue;
                }

                Vector2 extents = Vector2.zero;

                if (chart.isVertexMapped())
                {
                    // Let's assume vertex maps are arranged in a rectangle.
                    //ChartMesh mesh = chart.chartMesh();

                    // Arrange vertices in a rectangle.
                    extents.x = (float)chart.vertexMapWidth;
                    extents.y = (float)chart.vertexMapHeight;
                } else
                {
                    // Compute surface area to sort charts.
                    float chartArea = chart.computeSurfaceArea();
                    meshArea += chartArea;
                    //chartOrderArray[c] = chartArea;

                    // Compute chart scale
                    float parametricArea = Math.Abs(chart.computeParametricArea());    // @@ There doesn't seem to be anything preventing parametric area to be negative.
                    if (parametricArea < NV_EPSILON)
                    {
                        // When the parametric area is too small we use a rough approximation to prevent divisions by very small numbers.
                        Vector2 bounds = chart.computeParametricBounds();
                        parametricArea = bounds.x * bounds.y;
                    }
                    float scale = (chartArea / parametricArea) * texelsPerUnit;
                    if (parametricArea == 0) // < NV_EPSILON)
                    {
                        scale = 0;
                    }
                    Debug.Assert(isFinite(scale));

                    // Compute bounding box of chart.
                    computeBoundingBox(chart, out var majorAxis, out var minorAxis, out var origin, out var end);

                    Debug.Assert(isFinite(majorAxis) && isFinite(minorAxis) && isFinite(origin));

                    // Sort charts by perimeter. @@ This is sometimes producing somewhat unexpected results. Is this right?
                    //chartOrderArray[c] = ((end.x - origin.x) + (end.y - origin.y)) * scale;

                    // Translate, rotate and scale vertices. Compute extents.
                    ChartMesh mesh = chart.chartMesh();
                    var vertexCount = mesh.vertexLength;
                    for (var i = 0; i < vertexCount; i++)
                    {
                        Vertex vertex = mesh.vertexAt(i);

                        //Vector2 t = vertex.tex - origin;
                        Vector2 tmp;
                        tmp.x = Vector2.Dot(vertex.tex, majorAxis);
                        tmp.y = Vector2.Dot(vertex.tex, minorAxis);
                        tmp -= origin;
                        tmp *= scale;
                        if (tmp.x < 0 || tmp.y < 0)
                        {
                            Debug.Log($"tmp: {tmp.x} {tmp.y}");
                            Debug.Log($"scale: {scale}");
                            Debug.Log($"origin: {origin.x} {origin.y}");
                            Debug.Log($"majorAxis: {majorAxis.x} {majorAxis.y}");
                            Debug.Log($"minorAxis: {minorAxis.x} {minorAxis.y}");
                            Debug.DebugBreak();
                        }
                        //Debug.Assert(tmp.x >= 0 && tmp.y >= 0);

                        vertex.tex = tmp;

                        Debug.Assert(isFinite(vertex.tex));

                        extents = max(extents, tmp);
                    }
                    Debug.Assert(extents.x >= 0 && extents.y >= 0);

                    // Limit chart size.
                    if (extents.x > 1024 || extents.y > 1024)
                    {
                        float limit = Math.Max(extents.x, extents.y);

                        scale = 1024 / (limit + 1);

                        for (var i = 0; i < vertexCount; i++)
                        {
                            Vertex vertex = mesh.vertexAt(i);
                            vertex.tex *= scale;
                        }

                        extents *= scale;

                        Debug.Assert(extents.x <= 1024 && extents.y <= 1024);
                    }


                    // Scale the charts to use the entire texel area available. So, if the width is 0.1 we could scale it to 1 without increasing the lightmap usage and making a better 
                    // use of it. In many cases this also improves the look of the seams, since vertices on the chart boundaries have more chances of being aligned with the texel centers.

                    float scale_x = 1.0f;
                    float scale_y = 1.0f;

                    float divide_x = 1.0f;
                    float divide_y = 1.0f;

                    if (extents.x > 0)
                    {
                        int cw = Mathf.CeilToInt(extents.x);

                        if (blockAligned && chart.blockAligned)
                        {
                            // Align all chart extents to 4x4 blocks, but taking padding into account.
                            if (conservative)
                            {
                                cw = align(cw + 2, 4) - 2;
                            } else
                            {
                                cw = align(cw + 1, 4) - 1;
                            }
                        }

                        scale_x = ((float)(cw) - NV_EPSILON);
                        divide_x = extents.x;
                        extents.x = (float)(cw);
                    }

                    if (extents.y > 0)
                    {
                        int ch = Mathf.CeilToInt(extents.y);

                        if (blockAligned && chart.blockAligned)
                        {
                            // Align all chart extents to 4x4 blocks, but taking padding into account.
                            if (conservative)
                            {
                                ch = align(ch + 2, 4) - 2;
                            } else
                            {
                                ch = align(ch + 1, 4) - 1;
                            }
                        }

                        scale_y = ((float)(ch) - NV_EPSILON);
                        divide_y = extents.y;
                        extents.y = (float)(ch);
                    }

                    for (var v = 0; v < vertexCount; v++)
                    {
                        Vertex vertex = mesh.vertexAt(v);

                        vertex.tex.x /= divide_x;
                        vertex.tex.y /= divide_y;
                        vertex.tex.x *= scale_x;
                        vertex.tex.y *= scale_y;

                        Debug.Assert(isFinite(vertex.tex));
                    }
                }

                chartExtents[c] = extents;

                // Sort charts by perimeter.
                chartOrderArray[c] = extents.x + extents.y;
            }

            // @@ We can try to improve compression of small charts by sorting them by proximity like we do with vertex samples.
            // @@ How to do that? One idea: compute chart centroid, insert into grid, compute morton index of the cell, sort based on morton index.
            // @@ We would sort by morton index, first, then quantize the chart sizes, so that all small charts have the same size, and sort by size preserving the morton order.

            //Debug.Log("Sorting charts.\n");

            // Sort charts by area.
            m_radix = RadixSort.sort(chartOrderArray);
            var ranks = m_radix.ranks(); //uint[]

            // Estimate size of the map based on the mesh surface area and given texel scale.
            float texelCount = meshArea * square(texelsPerUnit) / 0.75f; // Assume 75% utilization.
            if (texelCount < 1) texelCount = 1;
            var approximateExtent = (int)nextPowerOfTwo((uint)(Math.Sqrt(texelCount)));


            // Init bit map.
            m_bitmap.clearAll();
            if (approximateExtent > m_bitmap.width())
            {
                m_bitmap.resize(approximateExtent, approximateExtent, false);
                //m_debug_bitmap.resize(approximateExtent, approximateExtent);
                //m_debug_bitmap.fill(new Color32(0,0,0,0));
            }


            int w = 0;
            int h = 0;


            // Add sorted charts to bitmap.
            for (var i = 0; i < chartCount; i++)
            {
                var c = ranks[chartCount - i - 1]; // largest chart first

                Chart chart = m_atlas.chartAt(c);

                if (!chart.isVertexMapped() && !chart.isDisk()) continue;

                //float scale_x = 1;
                //float scale_y = 1;

                var chart_bitmap = new BitMap();

                if (chart.isVertexMapped())
                {
                    chart.blockAligned = false;

                    // Init all bits to 1.
                    chart_bitmap.resize(Mathf.CeilToInt(chartExtents[c].x), Mathf.CeilToInt(chartExtents[c].y), /*initValue=*/true);

                    // @@ Another alternative would be to try to map each vertex to a different texel trying to fill all the available unused texels.
                } else
                {
                    // @@ Add special cases for dot and line charts. @@ Lightmap rasterizer also needs to handle these special cases.
                    // @@ We could also have a special case for chart quads. If the quad surface <= 4 texels, align vertices with texel centers and do not add padding. May be very useful for foliage.

                    // @@ In general we could reduce the padding of all charts by one texel by using a rasterizer that takes into account the 2-texel footprint of the tent bilinear filter. For example,
                    // if we have a chart that is less than 1 texel wide currently we add one texel to the left and one texel to the right creating a 3-texel-wide bitmap. However, if we know that the 
                    // chart is only 1 texel wide we could align it so that it only touches the footprint of two texels:

                    //      |   |      <- Touches texels 0, 1 and 2.
                    //    |   |        <- Only touches texels 0 and 1.
                    // \   \ / \ /   /
                    //  \   X   X   /
                    //   \ / \ / \ /
                    //    V   V   V
                    //    0   1   2

                    if (conservative)
                    {
                        // Init all bits to 0.
                        chart_bitmap.resize(Mathf.CeilToInt(chartExtents[c].x) + 2, Mathf.CeilToInt(chartExtents[c].y) + 2, /*initValue=*/false);  // + 2 to add padding on both sides.

                        // Rasterize chart and dilate.
                        drawChartBitmapDilate(chart, chart_bitmap, /*padding=*/1);
                    } else
                    {
                        // Init all bits to 0.
                        chart_bitmap.resize(Mathf.CeilToInt(chartExtents[c].x) + 1, Mathf.CeilToInt(chartExtents[c].y) + 1, /*initValue=*/false);  // Add half a texels on each side.

                        // Rasterize chart and dilate.
                        drawChartBitmap(chart, chart_bitmap, Vector2.one, new Vector2(0.5f, 0.5f));
                    }
                }

                findChartLocation(quality, chart_bitmap, chartExtents[c], w, h, out int best_x, out int best_y, out int best_cw, out int best_ch, out int best_r, chart.blockAligned);

                // Update parametric extents.
                w = Math.Max(w, best_x + best_cw);
                h = Math.Max(h, best_y + best_ch);

                w = align(w, 4);
                h = align(h, 4);

                // Resize bitmap if necessary.
                if ((w) > m_bitmap.width() || (h) > m_bitmap.height())
                {
                    //Debug.Log("Resize bitmap (%d, %d).\n", nextPowerOfTwo(w), nextPowerOfTwo(h));
                    m_bitmap.resize((int)nextPowerOfTwo((uint)(w)), (int)nextPowerOfTwo((uint)(h)), false);
                    //m_debug_bitmap.resize(nextPowerOfTwo((uint)(w)), nextPowerOfTwo((uint)(h)));
                }

                //Debug.Log("Add chart at (%d, %d).\n", best_x, best_y);

                addChart(chart_bitmap, w, h, best_x, best_y, best_r);//, /*debugOutput=*/null);

                // IC: Output chart again to debug bitmap.
                if (chart.isVertexMapped())
                {
                    addChart(chart_bitmap, w, h, best_x, best_y, best_r);//, m_debug_bitmap);
                } else
                {
                    addChart(chart, w, h, best_x, best_y, best_r);//, m_debug_bitmap);
                }

                //float best_angle = 2 * PI * best_r;

                // Translate and rotate chart texture coordinates.
                ChartMesh mesh = chart.chartMesh();
                var vertexCount = mesh.vertexLength;
                for (var v = 0; v < vertexCount; v++)
                {
                    Vertex vertex = mesh.vertexAt(v);

                    Vector2 t = vertex.tex;
                    if (best_r != 0) swap(ref t.x, ref t.y);
                    //vertex.tex.x = best_x + t.x * cosf(best_angle) - t.y * sinf(best_angle);
                    //vertex.tex.y = best_y + t.x * sinf(best_angle) + t.y * cosf(best_angle);

                    vertex.tex.x = best_x + t.x + 0.5f;
                    vertex.tex.y = best_y + t.y + 0.5f;

                    Debug.Assert(vertex.tex.x >= 0 && vertex.tex.y >= 0);
                    Debug.Assert(isFinite(vertex.tex.x));
                }
            }

            //w -= padding - 1; // Leave one pixel border!
            //h -= padding - 1;

            m_width = Math.Max(0, w);
            m_height = Math.Max(0, h);

            Debug.Assert(isAligned(m_width, 4));
            Debug.Assert(isAligned(m_height, 4));

            //m_debug_bitmap.resize(m_width, m_height);
            //m_debug_bitmap.setFormat(Image.Format_ARGB);
        }

        // IC: Brute force is slow, and random may take too much time to converge. We start inserting large charts in a small atlas. Using brute force is lame, because most of the space 
        // is occupied at this point. At the end we have many small charts and a large atlas with sparse holes. Finding those holes randomly is slow. A better approach would be to 
        // start stacking large charts as if they were tetris pieces. Once charts get small try to place them randomly. It may be interesting to try a intermediate strategy, first try 
        // along one axis and then try exhaustively along that axis.
        void findChartLocation(int quality, BitMap bitmap, Vector2 extents, int w, int h, out int best_x, out int best_y, out int best_w, out int best_h, out int best_r, bool blockAligned)
        {
            int attempts = 256;
            if (quality == 1) attempts = 4096;
            if (quality == 2) attempts = 2048;
            if (quality == 3) attempts = 1024;
            if (quality == 4) attempts = 512;

            if (quality == 0 || w * h < attempts)
            {
                findChartLocation_bruteForce(bitmap, /*extents,*/ w, h, out best_x, out best_y, out best_w, out best_h, out best_r, blockAligned);
            } else
            {
                findChartLocation_random(bitmap, /*extents,*/ w, h, out best_x, out best_y, out best_w, out best_h, out best_r, attempts, blockAligned);
            }
        }


        const int BLOCK_SIZE = 4;

        void findChartLocation_bruteForce(BitMap bitmap, /*Vector2 extents,*/ int w, int h, out int best_x, out int best_y, out int best_w, out int best_h, out int best_r, bool blockAligned)
        {
            int best_metric = int.MaxValue;

            int step_size = blockAligned ? BLOCK_SIZE : 1;

            best_x = 0;
            best_y = 0;
            best_h = 0;
            best_w = 0;
            best_r = 0;

            // Try two different orientations.
            for (int r = 0; r < 2; r++)
            {
                int cw = bitmap.width();
                int ch = bitmap.height();
                if ((r & 1) != 0) swap(ref cw, ref ch);

                for (int y = 0; y <= h + 1; y += step_size) // + 1 to extend atlas in case atlas full.
                {
                    for (int x = 0; x <= w + 1; x += step_size) // + 1 not really necessary here.
                    {
                        // Early out.
                        int area = Math.Max(w, x + cw) * Math.Max(h, y + ch);
                        //int perimeter = max(w, x+cw) + max(h, y+ch);
                        int extents = Math.Max(Math.Max(w, x + cw), Math.Max(h, y + ch));

                        int metric = extents * extents + area;

                        if (metric > best_metric)
                        {
                            continue;
                        }
                        if (metric == best_metric && Math.Max(x, y) >= Math.Max(best_x, best_y))
                        {
                            // If metric is the same, pick the one closest to the origin.
                            continue;
                        }

                        if (canAddChart(bitmap, w, h, x, y, r))
                        {
                            best_metric = metric;
                            best_x = x;
                            best_y = y;
                            best_w = cw;
                            best_h = ch;
                            best_r = r;

                            if (area == w * h)
                            {
                                // Chart is completely inside, do not look at any other location.
                                goto done;
                            }
                        }
                    }
                }
            }

done:
            Debug.Assert(best_metric != int.MaxValue);
        }


        void findChartLocation_random(BitMap bitmap, /*Vector2 extents,*/ int w, int h, out int best_x, out int best_y, out int best_w, out int best_h, out int best_r, int minTrialCount, bool blockAligned)
        {
            int best_metric = int.MaxValue;

            best_x = 0;
            best_y = 0;
            best_h = 0;
            best_w = 0;
            best_r = 0;

            for (int i = 0; i < minTrialCount || best_metric == int.MaxValue; i++)
            {
                int r = UnityEngine.Random.Range(0, 1);
                int x = UnityEngine.Random.Range(0, w + 1); // + 1 to extend atlas in case atlas full. We may want to use a higher number to increase probability of extending atlas.
                int y = UnityEngine.Random.Range(0, h + 1); // + 1 to extend atlas in case atlas full.

                if (blockAligned)
                {
                    x = align(x, BLOCK_SIZE);
                    y = align(y, BLOCK_SIZE);
                }

                int cw = bitmap.width();
                int ch = bitmap.height();
                if ((r & 1) != 0) swap(ref cw, ref ch);

                // Early out.
                int area = Math.Max(w, x + cw) * Math.Max(h, y + ch);
                //int perimeter = max(w, x+cw) + max(h, y+ch);
                int extents = Math.Max(Math.Max(w, x + cw), Math.Max(h, y + ch));

                int metric = extents * extents + area;

                if (metric > best_metric)
                {
                    continue;
                }
                if (metric == best_metric && Math.Min(x, y) > Math.Min(best_x, best_y))
                {
                    // If metric is the same, pick the one closest to the origin.
                    continue;
                }

                if (canAddChart(bitmap, w, h, x, y, r))
                {
                    best_metric = metric;
                    best_x = x;
                    best_y = y;
                    best_w = cw;
                    best_h = ch;
                    best_r = r;

                    if (area == w * h)
                    {
                        // Chart is completely inside, do not look at any other location.
                        break;
                    }
                }
            }
        }

        /*static*/
        bool checkBitsCallback(BitMap bitmap, int x, int y, Vector3 _0, Vector3 _1, Vector3 _2, float _3)
        {
            Debug.Assert(bitmap.bitAt(x, y) == false);

            return true;
        }

        /*static*/
        bool setBitsCallback(BitMap bitmap, int x, int y, Vector3 _0, Vector3 _1, Vector3 _2, float area)
        {
            if (area > 0.0)
            {
                bitmap.setBitAt(x, y);
            }

            return true;
        }

        void drawChartBitmapDilate(Chart chart, BitMap bitmap, int padding)
        {
            int w = bitmap.width();
            int h = bitmap.height();
            Vector2 extents = new Vector2((float)(w), (float)(h));

            Vector2[] vertices = new Vector2[4];

            // Rasterize chart faces, check that all bits are not set.
            var faceCount = chart.faceLength;
            for (int f = 0; f < faceCount; f++)
            {
                Face face = chart.chartMesh().faceAt(f);


                var edgeCount = 0;
                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    if (edgeCount < 4)
                    {
                        vertices[edgeCount] = it.vertex().tex + new Vector2(0.5f, 0.5f) + new Vector2((float)(padding), (float)(padding));
                    }
                    edgeCount++;
                }

                if (edgeCount == 3)
                {
                    Raster.drawTriangle(Mode.Antialiased, extents, true, vertices, setBitsCallback, bitmap);
                } else
                {
                    Raster.drawQuad(Mode.Antialiased, extents, true, vertices, setBitsCallback, bitmap);
                }
            }

            // Expand chart by padding pixels. (dilation)
            var tmp = new BitMap(w, h);
            for (int i = 0; i < padding; i++)
            {
                tmp.clearAll();

                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        bool b = bitmap.bitAt(x, y);
                        if (!b)
                        {
                            if (x > 0)
                            {
                                b |= bitmap.bitAt(x - 1, y);
                                if (y > 0) b |= bitmap.bitAt(x - 1, y - 1);
                                if (y < h - 1) b |= bitmap.bitAt(x - 1, y + 1);
                            }
                            if (y > 0) b |= bitmap.bitAt(x, y - 1);
                            if (y < h - 1) b |= bitmap.bitAt(x, y + 1);
                            if (x < w - 1)
                            {
                                b |= bitmap.bitAt(x + 1, y);
                                if (y > 0) b |= bitmap.bitAt(x + 1, y - 1);
                                if (y < h - 1) b |= bitmap.bitAt(x + 1, y + 1);
                            }
                        }
                        if (b) tmp.setBitAt(x, y);
                    }
                }

                swap(ref tmp, ref bitmap);
            }
        }

        static readonly Vector2[] pad = {
            new Vector2(-0.5f, -0.5f),
            new Vector2( 0.5f, -0.5f),
            new Vector2(-0.5f,  0.5f),
            new Vector2( 0.5f,  0.5f)
        };

        void drawChartBitmap(Chart chart, BitMap bitmap, Vector2 scale, Vector2 offset)
        {
            int w = bitmap.width();
            int h = bitmap.height();
            Vector2 extents = new Vector2((float)(w), (float)(h));

            Vector2[] vertices = new Vector2[4];

            // Rasterize 4 times to add proper padding.
            for (int i = 0; i < 4; i++)
            {

                // Rasterize chart faces, check that all bits are not set.
                var faceCount = chart.chartMesh().faceLength;
                for (var f = 0; f < faceCount; f++)
                {
                    Face face = chart.chartMesh().faceAt(f);


                    var edgeCount = 0;
                    for (var it = face.edges(); !it.isDone(); it.advance())
                    {
                        if (edgeCount < 4)
                        {
                            vertices[edgeCount] = it.vertex().tex * scale + offset + pad[i];
                            Debug.Assert(Mathf.CeilToInt(vertices[edgeCount].x) >= 0);
                            Debug.Assert(Mathf.CeilToInt(vertices[edgeCount].y) >= 0);
                            Debug.Assert(Mathf.CeilToInt(vertices[edgeCount].x) <= w);
                            Debug.Assert(Mathf.CeilToInt(vertices[edgeCount].y) <= h);
                        }
                        edgeCount++;
                    }

                    if (edgeCount == 3)
                    {
                        Raster.drawTriangle(Mode.Antialiased, extents, enableScissors: true, vertices, setBitsCallback, bitmap);
                    } else
                    {
                        Raster.drawQuad(Mode.Antialiased, extents, enableScissors: true, vertices, setBitsCallback, bitmap);
                    }
                }
            }

            // Expand chart by padding pixels. (dilation)
            var tmp = new BitMap(w, h);
            tmp.clearAll();

            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    bool b = bitmap.bitAt(x, y);
                    if (!b)
                    {
                        if (x > 0)
                        {
                            b |= bitmap.bitAt(x - 1, y);
                            if (y > 0) b |= bitmap.bitAt(x - 1, y - 1);
                            if (y < h - 1) b |= bitmap.bitAt(x - 1, y + 1);
                        }
                        if (y > 0) b |= bitmap.bitAt(x, y - 1);
                        if (y < h - 1) b |= bitmap.bitAt(x, y + 1);
                        if (x < w - 1)
                        {
                            b |= bitmap.bitAt(x + 1, y);
                            if (y > 0) b |= bitmap.bitAt(x + 1, y - 1);
                            if (y < h - 1) b |= bitmap.bitAt(x + 1, y + 1);
                        }
                    }
                    if (b) tmp.setBitAt(x, y);
                }
            }

            swap(ref tmp, ref bitmap);
        }

        bool canAddChart(BitMap bitmap, int atlas_w, int atlas_h, int offset_x, int offset_y, int r)
        {
            Debug.Assert(r == 0 || r == 1);

            // Check whether the two bitmaps overlap.

            int w = bitmap.width();
            int h = bitmap.height();

            if (r == 0)
            {
                for (int y = 0; y < h; y++)
                {
                    int yy = y + offset_y;
                    if (yy >= 0)
                    {
                        for (int x = 0; x < w; x++)
                        {
                            int xx = x + offset_x;
                            if (xx >= 0)
                            {
                                if (bitmap.bitAt(x, y))
                                {
                                    if (xx < atlas_w && yy < atlas_h)
                                    {
                                        if (m_bitmap.bitAt(xx, yy)) return false;
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (r == 1)
            {
                for (int y = 0; y < h; y++)
                {
                    int xx = y + offset_x;
                    if (xx >= 0)
                    {
                        for (int x = 0; x < w; x++)
                        {
                            int yy = x + offset_y;
                            if (yy >= 0)
                            {
                                if (bitmap.bitAt(x, y))
                                {
                                    if (xx < atlas_w && yy < atlas_h)
                                    {
                                        if (m_bitmap.bitAt(xx, yy)) return false;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return true;
        }


        static Color32 chartColor = new Color32(0, 0, 0, 0);
        static void selectRandomColor()
        {
            // Pick random color for this chart. @@ Select random hue, but fixed saturation/luminance?
            chartColor.r = (byte)(128 + UnityEngine.Random.Range(0, 127));
            chartColor.g = (byte)(128 + UnityEngine.Random.Range(0, 127));
            chartColor.b = (byte)(128 + UnityEngine.Random.Range(0, 127));
            chartColor.a = 255;
        }

        static bool debugDrawCallback(BitMap image, int x, int y, Vector3 _0, Vector3 _1, Vector3 _2, float area)
        {/*
            var image = (Image)param;

            if (area > 0.0)
            {
                Color32 c = image.pixel(x, y);
                c.r = chartColor.r;
                c.g = chartColor.g;
                c.b = chartColor.b;
                c.a += U8(ftoi_round(0.5f * area * 255));
                image.pixel(x, y) = c;
            }
            */
            return true;
        }


        void addChart(BitMap bitmap, int atlas_w, int atlas_h, int offset_x, int offset_y, int r)//, Image debugOutput)
        {
            Debug.Assert(r == 0 || r == 1);

            // Check whether the two bitmaps overlap.

            int w = bitmap.width();
            int h = bitmap.height();

            //if (debugOutput != null) {
            //    selectRandomColor();
            //}

            if (r == 0)
            {
                for (int y = 0; y < h; y++)
                {
                    int yy = y + offset_y;
                    if (yy >= 0)
                    {
                        for (int x = 0; x < w; x++)
                        {
                            int xx = x + offset_x;
                            if (xx >= 0)
                            {
                                if (bitmap.bitAt(x, y))
                                {
                                    if (xx < atlas_w && yy < atlas_h)
                                    {
                                        //if (debugOutput) debugOutput.pixel(xx, yy) = chartColor; else 
                                        {
                                            Debug.Assert(m_bitmap.bitAt(xx, yy) == false);
                                            m_bitmap.setBitAt(xx, yy);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (r == 1)
            {
                for (int y = 0; y < h; y++)
                {
                    int xx = y + offset_x;
                    if (xx >= 0)
                    {
                        for (int x = 0; x < w; x++)
                        {
                            int yy = x + offset_y;
                            if (yy >= 0)
                            {
                                if (bitmap.bitAt(x, y))
                                {
                                    if (xx < atlas_w && yy < atlas_h)
                                    {
                                        //if (debugOutput) debugOutput.pixel(xx, yy) = chartColor; else 
                                        {
                                            Debug.Assert(m_bitmap.bitAt(xx, yy) == false);
                                            m_bitmap.setBitAt(xx, yy);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }



        void addChart(Chart chart, int w, int h, int x, int y, int r)//, Image debugOutput)
        {
            Debug.Assert(r == 0 || r == 1);

            //Debug.Assert(debugOutput != null);
            //selectRandomColor();

            Vector2 extents = new Vector2((float)(w), (float)(h));
            Vector2 offset = new Vector2((float)(x), (float)(y)) + new Vector2(0.5f, 0.5f);

            Vector2[] vertices = new Vector2[4];

            // Rasterize chart faces, set bits.
            var faceCount = chart.faceLength;
            for (var f = 0; f < faceCount; f++)
            {
                var face = chart.chartMesh().faceAt(f);

                uint edgeCount = 0;
                for (var it = face.edges(); !it.isDone(); it.advance())
                {
                    if (edgeCount < 4)
                    {
                        Vector2 t = it.vertex().tex;
                        if (r == 1) swap(ref t.x, ref t.y);
                        vertices[edgeCount] = t + offset;
                    }
                    edgeCount++;
                }

                if (edgeCount == 3)
                {
                    Raster.drawTriangle(Mode.Antialiased, extents, /*enableScissors=*/true, vertices, debugDrawCallback, null);//, debugOutput);
                } else
                {
                    Raster.drawQuad(Mode.Antialiased, extents, /*enableScissors=*/true, vertices, debugDrawCallback, null);//, debugOutput);
                }
            }
        }

        /// A triangle for rasterization.
        class Triangle
        {
            public Triangle(Vector2 v0, Vector2 v1, Vector2 v2, Vector3 t0, Vector3 t1, Vector3 t2)
            {
                // Init vertices.
                this.v1 = v0;
                this.v2 = v2;
                this.v3 = v1;

                // Set barycentric coordinates.
                this.t1 = t0;
                this.t2 = t2;
                this.t3 = t1;

                // make sure every triangle is front facing.
                flipBackface();

                // Compute deltas.
                valid = computeDeltas();

                computeUnitInwardNormals();
            }

            public bool computeDeltas()
            {
                Vector2 e0 = v3 - v1;
                Vector2 e1 = v2 - v1;

                Vector3 de0 = t3 - t1;
                Vector3 de1 = t2 - t1;

                float denom = 1.0f / (e0.y * e1.x - e1.y * e0.x);
                if (!isFinite(denom))
                {
                    return false;
                }

                float lambda1 = -e1.y * denom;
                float lambda2 = e0.y * denom;
                float lambda3 = e1.x * denom;
                float lambda4 = -e0.x * denom;

                dx = de0 * lambda1 + de1 * lambda2;
                dy = de0 * lambda3 + de1 * lambda4;

                return true;
            }

            public bool draw(Vector2 extents, bool enableScissors, SamplingCallback cb, BitMap param)
            {
                // 28.4 fixed-point coordinates
                int Y1 = Mathf.RoundToInt(16.0f * v1.y);
                int Y2 = Mathf.RoundToInt(16.0f * v2.y);
                int Y3 = Mathf.RoundToInt(16.0f * v3.y);

                int X1 = Mathf.RoundToInt(16.0f * v1.x);
                int X2 = Mathf.RoundToInt(16.0f * v2.x);
                int X3 = Mathf.RoundToInt(16.0f * v3.x);

                // Deltas
                int DX12 = X1 - X2;
                int DX23 = X2 - X3;
                int DX31 = X3 - X1;

                int DY12 = Y1 - Y2;
                int DY23 = Y2 - Y3;
                int DY31 = Y3 - Y1;

                // Fixed-point deltas
                int FDX12 = DX12 << 4;
                int FDX23 = DX23 << 4;
                int FDX31 = DX31 << 4;

                int FDY12 = DY12 << 4;
                int FDY23 = DY23 << 4;
                int FDY31 = DY31 << 4;

                int minx, miny, maxx, maxy;
                if (enableScissors)
                {
                    int frustumX0 = 0 << 4;
                    int frustumY0 = 0 << 4;
                    int frustumX1 = (int)extents.x << 4;
                    int frustumY1 = (int)extents.y << 4;

                    // Bounding rectangle
                    minx = (Math.Max(Math.Min(X1, Math.Min(X2, X3)), frustumX0) + 0xF) >> 4;
                    miny = (Math.Max(Math.Min(Y1, Math.Min(Y2, Y3)), frustumY0) + 0xF) >> 4;
                    maxx = (Math.Min(Math.Max(X1, Math.Max(X2, X3)), frustumX1) + 0xF) >> 4;
                    maxy = (Math.Min(Math.Max(Y1, Math.Max(Y2, Y3)), frustumY1) + 0xF) >> 4;
                } else
                {
                    // Bounding rectangle
                    minx = (Math.Min(X1, Math.Min(X2, X3)) + 0xF) >> 4;
                    miny = (Math.Min(Y1, Math.Min(Y2, Y3)) + 0xF) >> 4;
                    maxx = (Math.Max(X1, Math.Max(X2, X3)) + 0xF) >> 4;
                    maxy = (Math.Max(Y1, Math.Max(Y2, Y3)) + 0xF) >> 4;
                }

                // Block size, standard 8x8 (must be power of two)
                const int q = 8;

                // @@ This won't work when minx,miny are negative. This code path is not used. Leaving as is for now.
                Debug.Assert(minx >= 0);
                Debug.Assert(miny >= 0);

                // Start in corner of 8x8 block
                minx &= ~(q - 1);
                miny &= ~(q - 1);

                // Half-edge constants
                int C1 = DY12 * X1 - DX12 * Y1;
                int C2 = DY23 * X2 - DX23 * Y2;
                int C3 = DY31 * X3 - DX31 * Y3;

                // Correct for fill convention
                if (DY12 < 0 || (DY12 == 0 && DX12 > 0)) C1++;
                if (DY23 < 0 || (DY23 == 0 && DX23 > 0)) C2++;
                if (DY31 < 0 || (DY31 == 0 && DX31 > 0)) C3++;

                // Loop through blocks
                for (int y = miny; y < maxy; y += q)
                {
                    for (int x = minx; x < maxx; x += q)
                    {
                        // Corners of block
                        int x0 = x << 4;
                        int x1 = (x + q - 1) << 4;
                        int y0 = y << 4;
                        int y1 = (y + q - 1) << 4;

                        // Evaluate half-space functions
                        var a00 = (C1 + DX12 * y0 - DY12 * x0 > 0) ? 1 : 0;
                        var a10 = (C1 + DX12 * y0 - DY12 * x1 > 0) ? 1 : 0;
                        var a01 = (C1 + DX12 * y1 - DY12 * x0 > 0) ? 1 : 0;
                        var a11 = (C1 + DX12 * y1 - DY12 * x1 > 0) ? 1 : 0;
                        int a = (a00 << 0) | (a10 << 1) | (a01 << 2) | (a11 << 3);

                        var b00 = (C2 + DX23 * y0 - DY23 * x0 > 0) ? 1 : 0;
                        var b10 = (C2 + DX23 * y0 - DY23 * x1 > 0) ? 1 : 0;
                        var b01 = (C2 + DX23 * y1 - DY23 * x0 > 0) ? 1 : 0;
                        var b11 = (C2 + DX23 * y1 - DY23 * x1 > 0) ? 1 : 0;
                        int b = (b00 << 0) | (b10 << 1) | (b01 << 2) | (b11 << 3);

                        var c00 = (C3 + DX31 * y0 - DY31 * x0 > 0) ? 1 : 0;
                        var c10 = (C3 + DX31 * y0 - DY31 * x1 > 0) ? 1 : 0;
                        var c01 = (C3 + DX31 * y1 - DY31 * x0 > 0) ? 1 : 0;
                        var c11 = (C3 + DX31 * y1 - DY31 * x1 > 0) ? 1 : 0;
                        int c = (c00 << 0) | (c10 << 1) | (c01 << 2) | (c11 << 3);

                        // Skip block when outside an edge
                        if (a == 0x0 || b == 0x0 || c == 0x0) continue;

                        // Accept whole block when totally covered
                        if (a == 0xF && b == 0xF && c == 0xF)
                        {
                            Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);

                            for (int iy = y; iy < y + q; iy++)
                            {
                                Vector3 tex = texRow;
                                for (int ix = x; ix < x + q; ix++)
                                {
                                    //Vector3 tex = t1 + dx * (ix - v1.x) + dy * (iy - v1.y);
                                    if (!cb(param, ix, iy, tex, dx, dy, 1.0f))
                                    {
                                        // early out.
                                        return false;
                                    }
                                    tex += dx;
                                }
                                texRow += dy;
                            }
                        } else // Partially covered block
                        {
                            int CY1 = C1 + DX12 * y0 - DY12 * x0;
                            int CY2 = C2 + DX23 * y0 - DY23 * x0;
                            int CY3 = C3 + DX31 * y0 - DY31 * x0;
                            Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);

                            for (int iy = y; iy < y + q; iy++)
                            {
                                int CX1 = CY1;
                                int CX2 = CY2;
                                int CX3 = CY3;
                                Vector3 tex = texRow;

                                for (int ix = x; ix < x + q; ix++)
                                {
                                    if (CX1 > 0 && CX2 > 0 && CX3 > 0)
                                    {
                                        if (!cb(param, ix, iy, tex, dx, dy, 1.0f))
                                        {
                                            // early out.
                                            return false;
                                        }
                                    }

                                    CX1 -= FDY12;
                                    CX2 -= FDY23;
                                    CX3 -= FDY31;
                                    tex += dx;
                                }

                                CY1 += FDX12;
                                CY2 += FDX23;
                                CY3 += FDX31;
                                texRow += dy;
                            }
                        }
                    }
                }

                return true;
            }


            readonly static float PX_INSIDE = 1.0f / (float)Math.Sqrt(2.0f);
            readonly static float PX_OUTSIDE = -1.0f / (float)Math.Sqrt(2.0f);

            const float BK_SIZE = 8;
            readonly static float BK_INSIDE = (float)Math.Sqrt(BK_SIZE * BK_SIZE / 2.0f);
            readonly static float BK_OUTSIDE = -(float)Math.Sqrt(BK_SIZE * BK_SIZE / 2.0f);

            public bool drawAA(Vector2 extents, bool enableScissors, SamplingCallback cb, BitMap param)
            {
                float minx, miny, maxx, maxy;
                if (enableScissors)
                {
                    // Bounding rectangle
                    minx = Mathf.Floor(Math.Max(Math.Min(v1.x, Math.Min(v2.x, v3.x)), 0.0f));
                    miny = Mathf.Floor(Math.Max(Math.Min(v1.y, Math.Min(v2.y, v3.y)), 0.0f));
                    maxx = Mathf.Ceil(Math.Min(Math.Max(v1.x, Math.Max(v2.x, v3.x)), extents.x - 1.0f));
                    maxy = Mathf.Ceil(Math.Min(Math.Max(v1.y, Math.Max(v2.y, v3.y)), extents.y - 1.0f));
                } else
                {
                    // Bounding rectangle
                    minx = Mathf.Floor(Math.Min(v1.x, Math.Min(v2.x, v3.x)));
                    miny = Mathf.Floor(Math.Min(v1.y, Math.Min(v2.y, v3.y)));
                    maxx = Mathf.Ceil(Math.Max(v1.x, Math.Max(v2.x, v3.x)));
                    maxy = Mathf.Ceil(Math.Max(v1.y, Math.Max(v2.y, v3.y)));
                }

                // There's no reason to align the blocks to the viewport, instead we align them to the origin of the triangle bounds.
                minx = Mathf.Floor(minx);
                miny = Mathf.Floor(miny);
                //minx = (float)(((int)minx) & (~((int)BK_SIZE - 1))); // align to blocksize (we don't need to worry about blocks partially out of viewport)
                //miny = (float)(((int)miny) & (~((int)BK_SIZE - 1)));

                minx += 0.5f; miny += 0.5f;  // sampling at texel centers!
                maxx += 0.5f; maxy += 0.5f;

                // Half-edge constants
                float C1 = n1.x * (-v1.x) + n1.y * (-v1.y);
                float C2 = n2.x * (-v2.x) + n2.y * (-v2.y);
                float C3 = n3.x * (-v3.x) + n3.y * (-v3.y);

                // Loop through blocks
                for (float y0 = miny; y0 <= maxy; y0 += BK_SIZE)
                {
                    for (float x0 = minx; x0 <= maxx; x0 += BK_SIZE)
                    {
                        // Corners of block
                        float xc = (x0 + (BK_SIZE - 1) / 2.0f);
                        float yc = (y0 + (BK_SIZE - 1) / 2.0f);

                        // Evaluate half-space functions
                        float aC = C1 + n1.x * xc + n1.y * yc;
                        float bC = C2 + n2.x * xc + n2.y * yc;
                        float cC = C3 + n3.x * xc + n3.y * yc;

                        // Skip block when outside an edge
                        if ((aC <= BK_OUTSIDE) || (bC <= BK_OUTSIDE) || (cC <= BK_OUTSIDE)) continue;

                        // Accept whole block when totally covered
                        if ((aC >= BK_INSIDE) && (bC >= BK_INSIDE) && (cC >= BK_INSIDE))
                        {
                            Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);

                            for (float y = y0; y < y0 + BK_SIZE; y++)
                            {
                                Vector3 tex = texRow;
                                for (float x = x0; x < x0 + BK_SIZE; x++)
                                {
                                    if (!cb(param, (int)x, (int)y, tex, dx, dy, 1.0f))
                                    {
                                        return false;
                                    }
                                    tex += dx;
                                }
                                texRow += dy;
                            }
                        } else // Partially covered block
                        {
                            float CY1 = C1 + n1.x * x0 + n1.y * y0;
                            float CY2 = C2 + n2.x * x0 + n2.y * y0;
                            float CY3 = C3 + n3.x * x0 + n3.y * y0;
                            Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);

                            for (float y = y0; y < y0 + BK_SIZE; y++) // @@ This is not clipping to scissor rectangle correctly.
                            {
                                float CX1 = CY1;
                                float CX2 = CY2;
                                float CX3 = CY3;
                                Vector3 tex = texRow;

                                for (float x = x0; x < x0 + BK_SIZE; x++)   // @@ This is not clipping to scissor rectangle correctly.
                                {
                                    if (CX1 >= PX_INSIDE && CX2 >= PX_INSIDE && CX3 >= PX_INSIDE)
                                    {
                                        // pixel completely covered
                                        Vector3 tex2 = t1 + dx * (x - v1.x) + dy * (y - v1.y);
                                        if (!cb(param, (int)x, (int)y, tex2, dx, dy, 1.0f))
                                        {
                                            return false;
                                        }
                                    } else if ((CX1 >= PX_OUTSIDE) && (CX2 >= PX_OUTSIDE) && (CX3 >= PX_OUTSIDE))
                                    {
                                        // triangle partially covers pixel. do clipping.
                                        var ct = new ClippedTriangle(v1 - new Vector2(x, y), v2 - new Vector2(x, y), v3 - new Vector2(x, y));
                                        ct.clipAABox(-0.5f, -0.5f, 0.5f, 0.5f);
                                        Vector2 centroid = ct.centroid();
                                        float area = ct.area();
                                        if (area > 0.0f)
                                        {
                                            Vector3 texCent = tex - dx * centroid.x - dy * centroid.y;
                                            //Debug.Assert(texCent.x >= -0.1f && texCent.x <= 1.1f); // @@ Centroid is not very exact...
                                            //Debug.Assert(texCent.y >= -0.1f && texCent.y <= 1.1f);
                                            //Debug.Assert(texCent.z >= -0.1f && texCent.z <= 1.1f);
                                            //Vector3 texCent2 = t1 + dx * (x - v1.x) + dy * (y - v1.y);
                                            if (!cb(param, (int)x, (int)y, texCent, dx, dy, area))
                                            {
                                                return false;
                                            }
                                        }
                                    }

                                    CX1 += n1.x;
                                    CX2 += n2.x;
                                    CX3 += n3.x;
                                    tex += dx;
                                }

                                CY1 += n1.y;
                                CY2 += n2.y;
                                CY3 += n3.y;
                                texRow += dy;
                            }
                        }
                    }
                }

                return true;
            }

            //public bool drawC(Vector2 extents, bool enableScissors, SamplingCallback cb, BitMap param) { throw new NotImplementedException(); }

            public void flipBackface()
            {
                // check if triangle is backfacing, if so, swap two vertices
                if (((v3.x - v1.x) * (v2.y - v1.y) - (v3.y - v1.y) * (v2.x - v1.x)) < 0)
                {
                    Vector2 hv = v1; v1 = v2; v2 = hv; // swap pos
                    Vector3 ht = t1; t1 = t2; t2 = ht; // swap tex
                }
            }
            public void computeUnitInwardNormals()
            {
                n1 = v1 - v2; n1 = new Vector2(-n1.y, n1.x); n1 = n1 * (1.0f / (float)Math.Sqrt(n1.x * n1.x + n1.y * n1.y));
                n2 = v2 - v3; n2 = new Vector2(-n2.y, n2.x); n2 = n2 * (1.0f / (float)Math.Sqrt(n2.x * n2.x + n2.y * n2.y));
                n3 = v3 - v1; n3 = new Vector2(-n3.y, n3.x); n3 = n3 * (1.0f / (float)Math.Sqrt(n3.x * n3.x + n3.y * n3.y));
            }

            // Vertices.	
            public Vector2 v1, v2, v3;
            public Vector2 n1, n2, n3; // unit inward normals
            public Vector3 t1, t2, t3;

            // Deltas.
            public Vector3 dx, dy;

            //public float sign;
            public bool valid;
        };

        class ClippedTriangle
        {
            public ClippedTriangle(Vector2 a, Vector2 b, Vector2 c)
            {
                m_numVertices = 3;
                m_activeVertexBuffer = 0;

                m_verticesA[0] = a;
                m_verticesA[1] = b;
                m_verticesA[2] = c;

                m_vertexBuffers[0] = m_verticesA;
                m_vertexBuffers[1] = m_verticesB;
            }

            public uint vertexCount()
            {
                return m_numVertices;
            }

            public Vector2[] vertices()
            {
                return m_vertexBuffers[m_activeVertexBuffer];
            }

            public void clipHorizontalPlane(float offset, float clipdirection)
            {
                Vector2[] v = m_vertexBuffers[m_activeVertexBuffer];
                m_activeVertexBuffer ^= 1;
                Vector2[] v2 = m_vertexBuffers[m_activeVertexBuffer];

                v[m_numVertices] = v[0];

                float dy2, dy1 = offset - v[0].y;
                int dy2in, dy1in = (clipdirection * dy1 >= 0) ? 1 : 0;
                uint p = 0;

                for (uint k = 0; k < m_numVertices; k++)
                {
                    dy2 = offset - v[k + 1].y;
                    dy2in = (clipdirection * dy2 >= 0) ? 1 : 0;

                    if (dy1in != 0) v2[p++] = v[k];

                    if (dy1in + dy2in == 1) // not both in/out
                    {
                        float dx = v[k + 1].x - v[k].x;
                        float dy = v[k + 1].y - v[k].y;
                        v2[p++] = new Vector2(v[k].x + dy1 * (dx / dy), offset);
                    }

                    dy1 = dy2; dy1in = dy2in;
                }
                m_numVertices = p;

                //for (uint k=0; k<m_numVertices; k++) printf("(%f, %f)\n", v2[k].x, v2[k].y); printf("\n");
            }

            public void clipVerticalPlane(float offset, float clipdirection)
            {
                Vector2[] v = m_vertexBuffers[m_activeVertexBuffer];
                m_activeVertexBuffer ^= 1;
                Vector2[] v2 = m_vertexBuffers[m_activeVertexBuffer];

                v[m_numVertices] = v[0];

                float dx2, dx1 = offset - v[0].x;
                int dx2in, dx1in = (clipdirection * dx1 >= 0) ? 1 : 0;
                uint p = 0;

                for (uint k = 0; k < m_numVertices; k++)
                {
                    dx2 = offset - v[k + 1].x;
                    dx2in = (clipdirection * dx2 >= 0 ? 1 : 0);

                    if (dx1in != 0) v2[p++] = v[k];

                    if (dx1in + dx2in == 1) // not both in/out
                    {
                        float dx = v[k + 1].x - v[k].x;
                        float dy = v[k + 1].y - v[k].y;
                        v2[p++] = new Vector2(offset, v[k].y + dx1 * (dy / dx));
                    }

                    dx1 = dx2; dx1in = dx2in;
                }
                m_numVertices = p;

                //for (uint k=0; k<m_numVertices; k++) printf("(%f, %f)\n", v2[k].x, v2[k].y); printf("\n");
            }

            public void computeAreaCentroid()
            {
                Vector2[] v = m_vertexBuffers[m_activeVertexBuffer];
                v[m_numVertices] = v[0];

                m_area = 0;
                float centroidx = 0, centroidy = 0;
                for (uint k = 0; k < m_numVertices; k++)
                {
                    // http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
                    float f = v[k].x * v[k + 1].y - v[k + 1].x * v[k].y;
                    m_area += f;
                    centroidx += f * (v[k].x + v[k + 1].x);
                    centroidy += f * (v[k].y + v[k + 1].y);
                }
                m_area = 0.5f * Math.Abs(m_area);
                if (m_area == 0)
                {
                    m_centroid = Vector2.zero;
                } else
                {
                    m_centroid = new Vector2(centroidx / (6 * m_area), centroidy / (6 * m_area));
                }
            }

            public void clipAABox(float x0, float y0, float x1, float y1)
            {
                clipVerticalPlane(x0, -1);
                clipHorizontalPlane(y0, -1);
                clipVerticalPlane(x1, 1);
                clipHorizontalPlane(y1, 1);

                computeAreaCentroid();
            }

            public Vector2 centroid()
            {
                return m_centroid;
            }

            public float area()
            {
                return m_area;
            }

            Vector2[] m_verticesA = new Vector2[7 + 1];
            Vector2[] m_verticesB = new Vector2[7 + 1];
            Vector2[][] m_vertexBuffers = new Vector2[2][];
            uint m_numVertices;
            uint m_activeVertexBuffer;
            float m_area;
            Vector2 m_centroid;
        };

        public enum Mode
        {
            Nearest,
            Antialiased,
            //Mode_Conservative
        };

        public delegate bool SamplingCallback(BitMap param, int x, int y, Vector3 bar, Vector3 dx, Vector3 dy, float coverage);

        class Raster
        {
            /// Process the given quad.
            public static bool drawQuad(Mode mode, Vector2 extents, bool enableScissors, Vector2[] v, SamplingCallback cb, BitMap param)
            {
                bool sign0 = triangleArea(v[0], v[1], v[2]) > 0.0f;
                bool sign1 = triangleArea(v[0], v[2], v[3]) > 0.0f;

                // Divide the quad into two non overlapping triangles.
                if (sign0 == sign1)
                {
                    Triangle tri0 = new Triangle(v[0], v[1], v[2], new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(1, 1, 0));
                    Triangle tri1 = new Triangle(v[0], v[2], v[3], new Vector3(0, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0));

                    if (tri0.valid && tri1.valid)
                    {
                        if (mode == Mode.Antialiased)
                        {
                            return tri0.drawAA(extents, enableScissors, cb, param) && tri1.drawAA(extents, enableScissors, cb, param);
                        } else
                        {
                            return tri0.draw(extents, enableScissors, cb, param) && tri1.draw(extents, enableScissors, cb, param);
                        }
                    }
                } else
                {
                    Triangle tri0 = new Triangle(v[0], v[1], v[3], new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0));
                    Triangle tri1 = new Triangle(v[1], v[2], v[3], new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0));

                    if (tri0.valid && tri1.valid)
                    {
                        if (mode == Mode.Antialiased)
                        {
                            return tri0.drawAA(extents, enableScissors, cb, param) && tri1.drawAA(extents, enableScissors, cb, param);
                        } else
                        {
                            return tri0.draw(extents, enableScissors, cb, param) && tri1.draw(extents, enableScissors, cb, param);
                        }
                    }
                }

                return true;
            }

            /// Process the given triangle.
            public static bool drawTriangle(Mode mode, Vector2 extents, bool enableScissors, Vector2[] v, SamplingCallback cb, BitMap param)
            {
                var tri = new Triangle(v[0], v[1], v[2], new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));

                // @@ It would be nice to have a conservative drawing mode that enlarges the triangle extents by one texel and is able to handle degenerate triangles.
                // @@ Maybe the simplest thing to do would be raster triangle edges.

                if (tri.valid)
                {
                    if (mode == Mode.Antialiased)
                    {
                        return tri.drawAA(extents, enableScissors, cb, param);
                    }
                    if (mode == Mode.Nearest)
                    {
                        return tri.draw(extents, enableScissors, cb, param);
                    }
                }

                return true;
            }
        }


        class RadixSort
        {
            // Constructor/Destructor
            public RadixSort() { m_size = 0; m_ranks = null; m_ranks2 = null; m_validRanks = false; }
            public RadixSort(int reserve_count)
            {
                m_size = 0; m_ranks = null; m_ranks2 = null; m_validRanks = false;
                checkResize(reserve_count);
            }

            // Invalidate ranks.
            public void reset() { m_validRanks = false; }

            public static RadixSort sort(float[] input)
            {
                var instance = new RadixSort();
                if (input == null || input.Length == 0) return instance;


                // Resize lists if needed
                instance.checkResize(input.Length);

                instance.insertionSort(input, input.Length);

                return instance;
            }



            unsafe void insertionSort(float[] input, int count)
            {
                if (!m_validRanks)
                {
                    /*for (uint i = 0; i < count; i++) {
                        m_ranks[i] = i;
                    }*/

                    m_ranks[0] = 0;
                    for (var i = 1; i != count; ++i)
                    {
                        var rank = m_ranks[i] = i;

                        var j = i;
                        while (j != 0 && input[rank] < input[m_ranks[j - 1]])
                        {
                            m_ranks[j] = m_ranks[j - 1];
                            --j;
                        }
                        if (i != j)
                        {
                            m_ranks[j] = rank;
                        }
                    }

                    m_validRanks = true;
                } else
                {
                    for (var i = 1; i != count; ++i)
                    {
                        var rank = m_ranks[i];

                        var j = i;
                        while (j != 0 && input[rank] < input[m_ranks[j - 1]])
                        {
                            m_ranks[j] = m_ranks[j - 1];
                            --j;
                        }
                        if (i != j)
                        {
                            m_ranks[j] = rank;
                        }
                    }
                }
            }

            // Sorting methods.
            //public RadixSort & sort(uint32 * input, uint count);
            //public RadixSort & sort(uint64 * input, uint count);
            //public RadixSort & sort(float * input, uint count);


            // Access to results. m_ranks is a list of indices in sorted order, i.e. in the order you may further process your data
            public int[] ranks() { Debug.Assert(m_validRanks); return m_ranks; }
            public int rank(uint i) { Debug.Assert(m_validRanks); return m_ranks[i]; }

            // query whether the sort has been performed
            public bool valid() { return m_validRanks; }

            int m_size;
            int[] m_ranks = new int[0];
            int[] m_ranks2 = new int[0];
            bool m_validRanks;

            // Internal methods
            //template <typename T> void insertionSort(T * input, uint count);
            //template <typename T> void radixSort(T * input, uint count);

            void resize(int count)
            {
                Array.Resize(ref m_ranks2, count);
                Array.Resize(ref m_ranks, count);
            }

            void checkResize(int count)
            {
                if (count != m_size)
                {
                    if (count > m_size) resize(count);
                    m_size = count;
                    m_validRanks = false;
                }
            }
        };
    }

    /// Fixed size vector class.
    public class FullVector
    {
        public FullVector(uint dim)
        {
            m_array = new float[dim];
        }
        public FullVector(FullVector v)
        {
            m_array = v.m_array.ToArray();
        }

        public uint dimension() { return (uint)m_array.Length; }

        public float this[int index] { get { return m_array[index]; } set { m_array[index] = value; } }

        public void fill(float f)
        {
            var dim = dimension();
            for (var i = 0; i < dim; i++)
            {
                m_array[i] = f;
            }
        }

        public static FullVector operator + (FullVector l, FullVector v)
        {
            Debug.Assert(l.dimension() == v.dimension());

            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] += v.m_array[i];
            }
            return dst;
        }

        public static FullVector operator - (FullVector l, FullVector v)
        {
            Debug.Assert(l.dimension() == v.dimension());

            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] -= v.m_array[i];
            }
            return dst;
        }

        public static FullVector operator * (FullVector l, FullVector v)
        {
            Debug.Assert(l.dimension() == v.dimension());

            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] *= v.m_array[i];
            }
            return dst;
        }

        public static FullVector operator + (FullVector l, float f)
        {
            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] += f;
            }
            return dst;
        }

        public static FullVector operator - (FullVector l, float f)
        {
            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] -= f;
            }
            return dst;
        }

        public static FullVector operator * (FullVector l, float f)
        {
            var dst = new FullVector(l);
            var dim = dst.dimension();
            for (var i = 0; i < dim; i++)
            {
                dst.m_array[i] *= f;
            }
            return dst;
        }



        float[] m_array;

    };

        
    /**
    * Sparse matrix class. The matrix is assumed to be sparse and to have
    * very few non-zero elements, for this reason it's stored in indexed 
    * format. To multiply column vectors efficiently, the matrix stores 
    * the elements in indexed-column order, there is a list of indexed 
    * elements for each row of the matrix. As with the FullVector the 
    * dimension of the matrix is constant.
    **/
    class SparseMatrix
    {

        // An element of the sparse array.
        public struct Coefficient 
        {
            public Coefficient(uint x, float v) { this.x = x; this.v = v; }
            public uint x;  // column
            public float v; // value
        };


        public SparseMatrix(uint d)
        {
            m_width = d;
            m_array = new List<Coefficient>[d];
            for (int n = 0; n < m_array.Length; n++)
                m_array[n] = new List<Coefficient>();
        }

        public SparseMatrix(uint w, uint h)
        {
            m_width = w;
            m_array = new List<Coefficient>[h];
            for (int n = 0; n < m_array.Length; n++)
                m_array[n] = new List<Coefficient>();
        }

        public SparseMatrix(SparseMatrix m)
        {
            m_width = m.m_width;
            m_array = new List<Coefficient>[m.m_array.Length];
            for (int n = 0; n < m.m_array.Length; n++)
                m_array[n] = m.m_array[n].ToList();
        }


        public uint width() { return m_width; }
        public uint height() { return (uint)m_array.Length; }
        public bool isSquare() { return width() == height(); }

        public float getCoefficient(uint x, uint y) // x is column, y is row
        {
            Debug.Assert(x < width());
            Debug.Assert(y < height());

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                if (m_array[y][i].x == x) return m_array[y][i].v;
            }

            return 0.0f;
        }

        public void setCoefficient(uint x, uint y, float f)
        {
            Debug.Assert(x < width());
            Debug.Assert(y < height());

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                if (m_array[y][i].x == x)
                {
                    var org = m_array[y][i];
                    org.v = f;
                    m_array[y][i] = org;
                    return;
                }
            }

            if (f != 0.0f)
            {
                var c = new Coefficient( x, f );
                m_array[y].Add(c);
            }
        }

        public void addCoefficient(uint x, uint y, float f)
        {
            Debug.Assert(x < width());
            Debug.Assert(y < height());

            if (f != 0.0f)
            {
                var count = m_array[y].Count;
                for (var i = 0; i < count; i++)
                {
                    if (m_array[y][i].x == x)
                    {
                        var org = m_array[y][i];
                        org.v += f;
                        m_array[y][i] = org;
                        return;
                    }
                }

                var c = new Coefficient( x, f );
                m_array[y].Add(c);
            }
        }

        public void mulCoefficient(uint x, uint y, float f)
        {
            Debug.Assert(x < width());
            Debug.Assert(y < height());

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                if (m_array[y][i].x == x)
                {
                    var org = m_array[y][i];
                    org.v *= f;
                    m_array[y][i] = org;
                    return;
                }
            }

            if (f != 0.0f)
            {
                var c = new Coefficient( x, f );
                m_array[y].Add(c);
            }
        }

        public float sumRow(uint y)
        {
            Debug.Assert( y < height() );

            var count = m_array[y].Count;

#if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < count; i++)
            {
                kahan.add(m_array[y][i].v);
            }
            return kahan.sum();
#else
            float sum = 0;
            for (var i = 0; i < count; i++)
            {
                sum += m_array[y][i].v;
            }
            return sum;
#endif
        }

        public float dotRow(uint y, FullVector v)
        {
            Debug.Assert( y < height() );

            var count = m_array[y].Count;

#if USE_KAHAN_SUM
            KahanSum kahan;
            for (uint i = 0; i < count; i++)
            {
                kahan.add(m_array[y][i].v * v[m_array[y][i].x]);
            }
            return kahan.sum();
#else
            float sum = 0;
            for (var i = 0; i < count; i++)
            {
                sum += m_array[y][i].v * v[(int)m_array[y][i].x];
            }
            return sum;
#endif
        }

        public void madRow(uint y, float alpha, FullVector v)
        {
            Debug.Assert(y < height());

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                v[(int)m_array[y][i].x] += alpha * m_array[y][i].v;
            }
        }

        public void clearRow(uint y)
        {
            Debug.Assert(y < height());

            m_array[y].Clear();
        }

        public void scaleRow(uint y, float f)
        {
            Debug.Assert(y < height());

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                var org = m_array[y][i];
                org.v *= f;
                m_array[y][i] = org;
            }
        }

        public void normalizeRow(uint y)
        {
            Debug.Assert(y < height());

            float norm = 0.0f;

            var count = m_array[y].Count;
            for (var i = 0; i < count; i++)
            {
                float f = m_array[y][i].v;
                norm += f * f;
            }

            scaleRow(y, 1.0f / Mathf.Sqrt(norm));
        }

        public void clearColumn(uint x)
        {
            Debug.Assert(x < width());

            for (var y = 0; y < height(); y++)
            {
                var count = m_array[y].Count;
                for (var e = 0; e < count; e++)
                {
                    if (m_array[y][e].x == x)
                    {
                        var org = m_array[y][e];
                        org.v = 0.0f;
                        m_array[y][e] = org;
                        break;
                    }
                }
            }
        }

        public void scaleColumn(uint x, float f)
        {
            Debug.Assert(x < width());

            for (var y = 0; y < height(); y++)
            {
                var count = m_array[y].Count;
                for (var e = 0; e < count; e++)
                {
                    if (m_array[y][e].x == x)
                    {
                        var org = m_array[y][e];
                        org.v *= f;
                        m_array[y][e] = org;
                        break;
                    }
                }
            }
        }

        public List<Coefficient> getRow(uint y)
        {
            return m_array[y];
        }

        public bool isSymmetric()
        {
            for (var y = 0; y < height(); y++)
            {
                var count = m_array[y].Count;
                for (var e = 0; e < count; e++)
                {
                    var x = m_array[y][e].x;
                    if (x > y)
                    {
                        float v = m_array[y][e].v;

                        if (!AtlasPacker.equal(getCoefficient((uint)y, x), v))
                        {  // @@ epsilon
                            return false;
                        }
                    }
                }
            }

            return true;
        }


        /// Number of columns.
        uint m_width;

        /// Array of matrix elements.
        List<Coefficient>[] m_array;
    };

    
    // Jacobi preconditioner.
    class JacobiPreconditioner //: public Preconditioner
    {

        public JacobiPreconditioner(SparseMatrix M, bool symmetric)
        {
            Debug.Assert(M.isSquare());

            m_inverseDiagonal = new FullVector(M.width());

            for (var x = 0; x < M.width(); x++)
            {
                float elem = M.getCoefficient((uint)x, (uint)x);
                //Debug.Assert( elem != 0.0f ); // This can be zero in the presence of zero area triangles.

                if (symmetric) 
                {
                    m_inverseDiagonal[x] = (elem != 0) ? 1.0f / Mathf.Sqrt(Mathf.Abs(elem)) : 1.0f;
                }
                else 
                {
                    m_inverseDiagonal[x] = (elem != 0) ? 1.0f / elem : 1.0f;
                }
            }
        }

        public void apply(FullVector x, FullVector y) 
        {
            Debug.Assert(x.dimension() == m_inverseDiagonal.dimension());
            Debug.Assert(y.dimension() == m_inverseDiagonal.dimension());

            // @@ Wrap vector component-wise product into a separate function.
            var D = x.dimension();
            for (var i = 0; i < D; i++)
            {
                y[i] = m_inverseDiagonal[i] * x[i];
            }
        }

        FullVector m_inverseDiagonal;

    };

    
    /// Basis class to compute tangent space basis, ortogonalizations and to
    /// transform vectors from one space to another.
    class Basis
    {
        /// Create a null basis.
        public Basis() { tangent = Vector3.zero; bitangent = Vector3.zero; normal = Vector3.zero; }

        /// Create a basis given three vectors.
        public Basis(Vector3 n, Vector3 t, Vector3 b) { tangent = t; bitangent = b; normal = n; }

        /// Create a basis with the given tangent vectors and the handness.
        public Basis(Vector3 n, Vector3 t, float sign)
        {
            build(n, t, sign);
        }

        const float NV_NORMAL_EPSILON = 0.001f;
        bool isNormalized(Vector3 v, float epsilon = NV_NORMAL_EPSILON)
        {
            return AtlasPacker.equal(v.magnitude, 1, epsilon);
        }

        Vector3 normalize(Vector3 v, float epsilon = NV_EPSILON)
        {
            float l = v.magnitude;
            Debug.Assert(!Face.isZero(l, epsilon));
            Vector3 n = Face.scale(v, 1.0f / l);
            Debug.Assert(isNormalized(n));
            return n;
        }

        const float NV_EPSILON = 0.0001f;
        public void normalize(float epsilon = NV_EPSILON)
        {
            normal = Face.normalizeSafe(normal, Vector3.zero, epsilon);
            tangent = Face.normalizeSafe(tangent, Vector3.zero, epsilon);
            bitangent = Face.normalizeSafe(bitangent, Vector3.zero, epsilon);
        }

        public void orthonormalize(float epsilon = NV_EPSILON)
        {
            // N' = |N|
            // T' = |T - (N' dot T) N'|
            // B' = |B - (N' dot B) N' - (T' dot B) T'|

            normal = normalize(normal, epsilon);

            tangent -= normal * Vector3.Dot(normal, tangent);
            tangent = normalize(tangent, epsilon);

            bitangent -= normal * Vector3.Dot(normal, bitangent);
            bitangent -= tangent * Vector3.Dot(tangent, bitangent);
            bitangent = normalize(bitangent, epsilon);
        }

        float lengthSquared(Vector3 v)
        {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        }

        public void robustOrthonormalize(float epsilon = NV_EPSILON)
        {
            // Normalize all vectors.
            normalize(epsilon);

            if (lengthSquared(normal) < epsilon*epsilon)
            {
                // Build normal from tangent and bitangent.
                normal = Vector3.Cross(tangent, bitangent);

                if (lengthSquared(normal) < epsilon*epsilon)
                {
                    // Arbitrary basis.
                    tangent   = new Vector3(1, 0, 0);
                    bitangent = new Vector3(0, 1, 0);
                    normal    = new Vector3(0, 0, 1);
                    return;
                }

                normal = normalize(normal, epsilon);
            }

            // Project tangents to normal plane.
            tangent -= normal * Vector3.Dot(normal, tangent);
            bitangent -= normal * Vector3.Dot(normal, bitangent);

            if (lengthSquared(tangent) < epsilon*epsilon)
            {
                if (lengthSquared(bitangent) < epsilon*epsilon)
                {
                    // Arbitrary basis.
                    buildFrameForDirection(normal);
                }
                else
                {
                    // Build tangent from bitangent.
                    bitangent = normalize(bitangent, epsilon);

                    tangent = Vector3.Cross(bitangent, normal);
                    Debug.Assert(isNormalized(tangent, epsilon));
                }
            }
            else
            {
                tangent = normalize(tangent, epsilon);

                if (lengthSquared(bitangent) < epsilon*epsilon)
                {
                    // Build bitangent from tangent.
                    bitangent = Vector3.Cross(tangent, normal);
                    Debug.Assert(isNormalized(bitangent, epsilon));
                }
                else
                {
                    bitangent = normalize(bitangent, epsilon);

                    // At this point tangent and bitangent are orthogonal to normal, but we don't know whether their orientation.
            
                    Vector3 bisector;
                    if (lengthSquared(tangent + bitangent) < epsilon*epsilon)
                    {
                        bisector = tangent;
                    }
                    else
                    {
                        bisector = normalize(tangent + bitangent);
                    }
                    Vector3 axis = normalize(Vector3.Cross(bisector, normal));

                    //Debug.Assert(isNormalized(axis, epsilon));
                    Debug.Assert(AtlasPacker.equal(Vector3.Dot(axis, tangent), -Vector3.Dot(axis, bitangent), epsilon));

                    if (Vector3.Dot(axis, tangent) > 0)
                    {
                        tangent = bisector + axis;
                        bitangent = bisector - axis;
                    }
                    else
                    {
                        tangent = bisector - axis;
                        bitangent = bisector + axis;
                    }

                    // Make sure the resulting tangents are still perpendicular to the normal.
                    tangent -= normal * Vector3.Dot(normal, tangent);
                    bitangent -= normal * Vector3.Dot(normal, bitangent);

                    // Double check.
                    Debug.Assert(AtlasPacker.equal(Vector3.Dot(normal, tangent), 0.0f, epsilon));
                    Debug.Assert(AtlasPacker.equal(Vector3.Dot(normal, bitangent), 0.0f, epsilon));

                    // Normalize.
                    tangent = normalize(tangent);
                    bitangent = normalize(bitangent);

                    // If tangent and bitangent are not orthogonal, then derive bitangent from tangent, just in case...
                    if (!AtlasPacker.equal(Vector3.Dot(tangent, bitangent), 0.0f, epsilon)) {
                        bitangent = Vector3.Cross(tangent, normal);
                        bitangent = normalize(bitangent);
                    }
                }
            }

            /*// Check vector lengths.
            if (!isNormalized(normal, epsilon))
            {
                Debug.Log("%f %f %f\n", normal.x, normal.y, normal.z);
                Debug.Log("%f %f %f\n", tangent.x, tangent.y, tangent.z);
                Debug.Log("%f %f %f\n", bitangent.x, bitangent.y, bitangent.z);
            }*/

            Debug.Assert(isNormalized(normal, epsilon));
            Debug.Assert(isNormalized(tangent, epsilon));
            Debug.Assert(isNormalized(bitangent, epsilon));

            // Check vector angles.
            Debug.Assert(AtlasPacker.equal(Vector3.Dot(normal, tangent), 0.0f, epsilon));
            Debug.Assert(AtlasPacker.equal(Vector3.Dot(normal, bitangent), 0.0f, epsilon));
            Debug.Assert(AtlasPacker.equal(Vector3.Dot(tangent, bitangent), 0.0f, epsilon));

            // Check vector orientation.
            var det = Vector3.Dot(Vector3.Cross(normal, tangent), bitangent);
            Debug.Assert(AtlasPacker.equal(det, 1.0f, epsilon) || AtlasPacker.equal(det, -1.0f, epsilon));
        }

        public void buildFrameForDirection(Vector3 d, float angle = 0)
        {
            Debug.Assert(isNormalized(d));
            normal = d;

            // Choose minimum axis.
            if (Mathf.Abs(normal.x) < Mathf.Abs(normal.y) && Mathf.Abs(normal.x) < Mathf.Abs(normal.z))
            {
                tangent = new Vector3(1, 0, 0);
            } else if (Mathf.Abs(normal.y) < Mathf.Abs(normal.z))
            {
                tangent = new Vector3(0, 1, 0);
            } else
            {
                tangent = new Vector3(0, 0, 1);
            }

            // Ortogonalize
            tangent -= normal * Vector3.Dot(normal, tangent);
            tangent = normalize(tangent);

            bitangent = Vector3.Cross(normal, tangent);

            // Rotate frame around normal according to angle.
            if (angle != 0.0f)
            {
                float c = Mathf.Cos(angle);
                float s = Mathf.Sin(angle);
                Vector3 tmp = c * tangent - s * bitangent;
                bitangent = s * tangent + c * bitangent;
                tangent = tmp;
            }
        }

        /// Calculate the determinant [ F G N ] to obtain the handness of the basis. 
        public float handness() 
        {
            return determinant() > 0.0f ? 1.0f : -1.0f;
        }

        /// Build a basis from 2 vectors and a handness flag.
        public void build(Vector3 n, Vector3 t, float sign)
        {
            normal = n;
            tangent = t;
            bitangent = sign * Vector3.Cross(t, n);
        }

        /// Compute the determinant of this basis.
        public float determinant() 
        {
            return 
                tangent.x * bitangent.y * normal.z - tangent.z * bitangent.y * normal.x +
                tangent.y * bitangent.z * normal.x - tangent.y * bitangent.x * normal.z + 
                tangent.z * bitangent.x * normal.y - tangent.x * bitangent.z * normal.y;
        }

        public bool isValid()
        {
            if (AtlasPacker.equal(normal, Vector3.zero)) return false;
            if (AtlasPacker.equal(tangent, Vector3.zero)) return false;
            if (AtlasPacker.equal(bitangent, Vector3.zero)) return false;

            if (AtlasPacker.equal(determinant(), 0.0f)) return false;

            return true;
        }

        // Get transform matrix for this basis.
        //public  Matrix matrix() ;

        // Transform by this basis. (From this basis to object space).
        public  Vector3 transform(Vector3 v)
        {
            Vector3 o = tangent * v.x;
            o += bitangent * v.y;
            o += normal * v.z;
            return o;
        }

        // Transform by the transpose. (From object space to this basis).
        public  Vector3 transformT(Vector3 v)
        {
            return new Vector3(Vector3.Dot(tangent, v), Vector3.Dot(bitangent, v), Vector3.Dot(normal, v));
        }

        // Transform by the inverse. (From object space to this basis).
        public  Vector3 transformI(Vector3 v)
        {
            var det = determinant();
            Debug.Assert(!AtlasPacker.equal(det, 0.0f, 0.0f));

            var idet = 1.0f / det;

            // Rows of the inverse matrix.
            Vector3 r0 = new Vector3(
                 (bitangent.y * normal.z - bitangent.z * normal.y),
                -(bitangent.x * normal.z - bitangent.z * normal.x),
                 (bitangent.x * normal.y - bitangent.y * normal.x));

            Vector3 r1 = new Vector3(
                -(tangent.y * normal.z - tangent.z * normal.y),
                 (tangent.x * normal.z - tangent.z * normal.x),
                -(tangent.x * normal.y - tangent.y * normal.x));

            Vector3 r2 = new Vector3(
                 (tangent.y * bitangent.z - tangent.z * bitangent.y),
                -(tangent.x * bitangent.z - tangent.z * bitangent.x),
                 (tangent.x * bitangent.y - tangent.y * bitangent.x));

            return new Vector3(Vector3.Dot(v, r0), Vector3.Dot(v, r1), Vector3.Dot(v, r2)) * idet;
        }


        public Vector3 tangent;
        public Vector3 bitangent;
        public Vector3 normal;
    };


    public struct Cell
    {
        public List<uint> indexArray;
    };

    public class ProximityGrid
    {
        public void reset()
        {
            if (cellArray == null)
                return;
            for (int i = 0; i < cellArray.Length; i++)
                cellArray[i].indexArray.Clear();
        }

        public void init(List<Vector3> pointArray)
        {
            // Compute bounding box.
            var box = new MinMaxAABB();

            var count = pointArray.Count;
            if (count > 0)
            {
                {
                    var point = pointArray[0];
                    box.Min = point;
                    box.Max = point;
                }
                for (var i = 1; i < count; i++)
                {
                    var point = pointArray[i];
                    box.Min.x = Math.Min(box.Min.x, point.x);
                    box.Min.y = Math.Min(box.Min.y, point.y);
                    box.Min.z = Math.Min(box.Min.z, point.z);

                    box.Max.x = Math.Max(box.Max.x, point.x);
                    box.Max.y = Math.Max(box.Max.y, point.y);
                    box.Max.z = Math.Max(box.Max.z, point.z);
                }
            }

            init(box, count);

            // Insert all points.
            for (var i = 0; i < count; i++)
            {
                add(pointArray[i], (uint)i);
            }
        }

        public void init(MinMaxAABB box, int count)
        {
            reset();

            // Determine grid size.
            float cellWidth;

            Vector3 diagonal = (box.Max - box.Min);
            Vector3 extents = diagonal * 0.5f;
            float volume = 8.0f * (extents.x * extents.y * extents.z);

            if (AtlasPacker.equal(volume, 0))
            {
                // Degenerate box, treat like a quad.
                Vector2 quad;
                if (diagonal.x < diagonal.y && diagonal.x < diagonal.z) {
                    quad.x = diagonal.y;
                    quad.y = diagonal.z;
                } else if (diagonal.y < diagonal.x && diagonal.y < diagonal.z) {
                    quad.x = diagonal.x;
                    quad.y = diagonal.z;
                } else {
                    quad.x = diagonal.x;
                    quad.y = diagonal.y;
                }

                float cellArea = quad.x * quad.y / count;
                cellWidth = Mathf.Sqrt(cellArea); // pow(cellArea, 1.0f / 2.0f);
            } else {
                // Ideally we want one cell per point.
                float cellVolume = volume / count;
                cellWidth = Mathf.Pow(cellVolume, 1.0f / 3.0f);
            }

            Debug.Assert(cellWidth != 0);

            sx = Mathf.Max(1, Mathf.CeilToInt(diagonal.x / cellWidth));
            sy = Mathf.Max(1, Mathf.CeilToInt(diagonal.y / cellWidth));
            sz = Mathf.Max(1, Mathf.CeilToInt(diagonal.z / cellWidth));

            invCellSize.x = (float)(sx) / diagonal.x;
            invCellSize.y = (float)(sy) / diagonal.y;
            invCellSize.z = (float)(sz) / diagonal.z;

            cellArray = new Cell[sx * sy * sz];
            for (int i = 0; i < cellArray.Length; i++)
                cellArray[i].indexArray = new List<uint>();

            corner = box.Min; // @@ Align grid better?
        }

        public int index_x(float x)
        {
            return Mathf.Clamp(Mathf.FloorToInt((x - corner.x) * invCellSize.x), 0, sx - 1);
        }

        public int index_y(float y)
        {
            return Mathf.Clamp(Mathf.FloorToInt((y - corner.y) * invCellSize.y), 0, sy - 1);
        }
        public int index_z(float z)
        {
            return Mathf.Clamp(Mathf.FloorToInt((z - corner.z) * invCellSize.z), 0, sz - 1);
        }
        public int index(int x, int y, int z)
        {
            Debug.Assert(x >= 0 && x < sx);
            Debug.Assert(y >= 0 && y < sy);
            Debug.Assert(z >= 0 && z < sz);
            int idx = (z * sy + y) * sx + x;
            Debug.Assert(idx >= 0 && (uint)(idx) < cellArray.Length);
            return idx;
        }

        public int index(Vector3 pos)
        {
            int x = index_x(pos.x);
            int y = index_y(pos.y);
            int z = index_z(pos.z);
            return index(x, y, z);
        }

        public uint mortonCount()
        {
            var s = (long)(Math.Max(sx, Math.Max(sy, sz)));
            s = AtlasPacker.nextPowerOfTwo(s);

            if (s > 1024) {
                return (uint)(s * s * Math.Max(sx, Math.Max(sy, sz)));
            }

            return (uint)(s * s * s);
        }

        public int mortonIndex(uint code)
        {
            uint x, y, z;

            uint s = (uint)(Math.Max(sx, Math.Max(sy, sz)));
            if (s > 1024)
            {
                // Use layered two-dimensional morton order.
                s = AtlasPacker.nextPowerOfTwo(s);
                uint layer = code / (s * s);
                code = code % (s * s);

                uint layer_count = (uint)(Math.Min(sx, Math.Min(sy, sz)));
                if (sx == layer_count)
                {
                    x = layer;
                    y = Chart.decodeMorton2X(code);
                    z = Chart.decodeMorton2Y(code);
                } else if (sy == layer_count)
                {
                    x = Chart.decodeMorton2Y(code);
                    y = layer;
                    z = Chart.decodeMorton2X(code);
                } else /*if (sz == layer_count)*/
                {
                    x = Chart.decodeMorton2X(code);
                    y = Chart.decodeMorton2Y(code);
                    z = layer;
                }
            } else
            {
                x = Chart.decodeMorton3X(code);
                y = Chart.decodeMorton3Y(code);
                z = Chart.decodeMorton3Z(code);
            }

            if (x >= (uint)(sx) || y >= (uint)(sy) || z >= (uint)(sz))
            {
                return -1;
            }

            return index((int)x, (int)y, (int)z);
        }


        public void add(Vector3 pos, uint key)
        {
            var idx = index(pos);
            cellArray[idx].indexArray.Add(key);
        }
        public bool remove(Vector3 pos, uint key)
        {
            var idx = index(pos);
            return cellArray[idx].indexArray.Remove(key);
        }

        public void gather(Vector3 position, float radius, List<uint> indexArray)
        {
            int x0 = index_x(position.x - radius);
            int x1 = index_x(position.x + radius);

            int y0 = index_y(position.y - radius);
            int y1 = index_y(position.y + radius);

            int z0 = index_z(position.z - radius);
            int z1 = index_z(position.z + radius);

            for (int z = z0; z <= z1; z++) {
                for (int y = y0; y <= y1; y++) {
                    for (int x = x0; x <= x1; x++) {
                        int idx = index(x, y, z);
                        indexArray.AddRange(cellArray[idx].indexArray);
                    }
                }
            }
        }

        public Cell[] cellArray;

        public Vector3 corner;
        public Vector3 invCellSize;
        public int sx, sy, sz;
    };
    
    class AtlasBuilder
    {
        public AtlasBuilder(ChartMesh m)
        {
            mesh = m;
            var faceCount = m.faceLength;
            faceChartArray = new int[faceCount];
            for (int i = 0; i < faceCount; i++)
                faceChartArray[i] = -1;

            faceCandidateArray = new uint[faceCount];
            for (int i = 0; i < faceCount; i++)
                faceCandidateArray[i] = unchecked((uint)-1);

            // @@ Floyd for the whole mesh is too slow. We could compute floyd progressively per patch as the patch grows. We need a better solution to compute most central faces.
            //computeShortestPaths();

            // Precompute edge lengths and face areas.
            var edgeCount = m.edgeLength;
            edgeLengths = new float[edgeCount];

            for (var i = 0; i < edgeCount; i++)
            {
                var id = m.edgeAt(i).id;
                Debug.Assert(id / 2 == i);

                edgeLengths[i] = m.edgeAt(i).length();
            }

            faceAreas = new float[faceCount];
            for (var i = 0; i < faceCount; i++)
            {
                faceAreas[i] = m.faceAt(i).area();
            }
        }

        public void markUnchartedFaces(List<uint> unchartedFaces)
        {
            var unchartedFaceCount = unchartedFaces.Count;
            for (var i = 0; i < unchartedFaceCount; i++)
            {
                var f = unchartedFaces[i];
                faceChartArray[(int)f] = -2;
                //faceCandidateArray[f] = -2; // @@ ?

                removeCandidate(f);
            }

            Debug.Assert(facesLeft >= unchartedFaceCount);
            facesLeft -= (uint)unchartedFaceCount;
        }

        public void computeShortestPaths()
        {
            var faceCount = mesh.faceLength;
            var shortestPaths = new float[faceCount * faceCount];
            for (int i = 0; i < shortestPaths.Length; i++)
                shortestPaths[i] = float.MaxValue;

            // Fill edges:
            for (var i = 0; i < faceCount; i++)
            {
                shortestPaths[i * faceCount + i] = 0.0f;

                var face_i = mesh.faceAt(i);
                Vector3 centroid_i = face_i.centroid();

                for (var it = face_i.edges(); !it.isDone(); it.advance())
                {
                    var edge = it.current();

                    if (!edge.isBoundary())
                    {
                        var face_j = edge.pair.face;

                        var j = (int)face_j.id;
                        var centroid_j = face_j.centroid();

                        shortestPaths[i * faceCount + j] = shortestPaths[j * faceCount + i] = (centroid_i - centroid_j).magnitude;
                    }
                }
            }

            // Use Floyd-Warshall algorithm to compute all paths:
            for (var k = 0; k < faceCount; k++)
            {
                for (var i = 0; i < faceCount; i++)
                {
                    for (var j = 0; j < faceCount; j++)
                    {
                        shortestPaths[i * faceCount + j] = Math.Min(shortestPaths[i * faceCount + j], shortestPaths[i * faceCount + k] + shortestPaths[k * faceCount + j]);
                    }
                }
            }
        }

        public void placeSeeds(float threshold, uint maxSeedCount)
        {
            // Instead of using a predefiened number of seeds:
            // - Add seeds one by one, growing chart until a certain treshold.
            // - Undo charts and restart growing process.

            // @@ How can we give preference to faces far from sharp features as in the LSCM paper?
            //   - those points can be found using a simple flood filling algorithm.
            //   - how do we weight the probabilities?

            for (uint i = 0; i < maxSeedCount; i++)
            {
                if (facesLeft == 0)
                {
                    // No faces left, stop creating seeds.
                    break;
                }

                createRandomChart(threshold);
            }
        }

        public void createRandomChart(float threshold)
        {
            var chart = new ChartBuildData(chartArray.Count);
            chartArray.Add(chart);

            // Pick random face that is not used by any chart yet.
            var randomFaceIdx = UnityEngine.Random.Range(0, facesLeft - 1);
            var i = 0;
            for (var f = 0; f != randomFaceIdx; f++, i++)
            {
                while (faceChartArray[i] != -1) i++;
            }
            while (faceChartArray[i] != -1) i++;

            chart.seeds.Add((uint)i);

            addFaceToChart(chart, (uint)i, true);

            // Grow the chart as much as possible within the given threshold.
            growChart(chart, threshold * 0.5f, facesLeft);
            //growCharts(threshold - threshold * 0.75f / chartCount(), facesLeft);
        }

        public void addFaceToChart(ChartBuildData chart, uint f, bool recomputeProxy=false)
        {
            // Add face to chart.
            chart.faces.Add(f);

            Debug.Assert(faceChartArray[(int)f] == -1);
            faceChartArray[(int)f] = chart.id;

            facesLeft--;

            // Update area and boundary length.
            chart.area = evaluateChartArea(chart, f);
            chart.boundaryLength = evaluateBoundaryLength(chart, f);
            chart.normalSum = evaluateChartNormalSum(chart, f);
            chart.centroidSum = evaluateChartCentroidSum(chart, f);

            if (recomputeProxy)
            {
                // Update proxy and candidate's priorities.
                updateProxy(chart);
            }

            // Update candidates.
            removeCandidate(f);
            updateCandidates(chart, f);
            updatePriorities(chart);
        }

        public bool growCharts(float threshold, uint faceCount)
        {
            // Using one global list.

            faceCount = Math.Min(faceCount, facesLeft);

            for (uint i = 0; i < faceCount; i++)
            {
                var candidate = getBestCandidate();
        
                if (candidate.metric > threshold) {
                    return false; // Can't grow more.
                }

                addFaceToChart(candidate.chart, candidate.face);
            }

            return facesLeft != 0; // Can continue growing.
        }

        public bool growChart(ChartBuildData chart, float threshold, uint faceCount)
        {
            // Try to add faceCount faces within threshold to chart.
            for (uint i = 0; i < faceCount; )
            {
                if (chart.candidates.Count == 0 || chart.candidates.firstPriority() > threshold)
                {
                    return false;
                }

                uint f = chart.candidates.pop();
                if (faceChartArray[(int)f] == -1)
                {
                    addFaceToChart(chart, f);
                    i++;
                }
            }

            if (chart.candidates.Count == 0 || chart.candidates.firstPriority() > threshold)
            {
                return false;
            }

            return true;
        }

        public void resetCharts()
        {
            var faceCount = mesh.faceLength;
            for (var i = 0; i < faceCount; i++)
            {
                faceChartArray[i] = -1;
                faceCandidateArray[i] = unchecked ((uint)-1);
            }

            facesLeft = (uint)faceCount;

            candidateArray.Clear();

            var chartCount = chartArray.Count;
            for (var i = 0; i < chartCount; i++)
            {
                var chart = chartArray[i];

                var seed = chart.seeds[chart.seeds.Count - 1];

                chart.area = 0.0f;
                chart.boundaryLength = 0.0f;
                chart.normalSum = Vector3.zero;
                chart.centroidSum = Vector3.zero;

                chart.faces.Clear();
                chart.candidates.Clear();

                addFaceToChart(chart, seed);
            }
        }

        public void updateCandidates(ChartBuildData chart, uint f)
        {
            var face = mesh.faceAt((int)f);

            // Traverse neighboring faces, add the ones that do not belong to any chart yet.
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current().pair;

                if (!edge.isBoundary())
                {
                    var fid = edge.face.id;

                    if (faceChartArray[(int)fid] == -1)
                    {
                        chart.candidates.push(fid);
                    }
                }
            }
        }

        public void updateProxies()
        {
            var chartCount = chartArray.Count;
            for (var i = 0; i < chartCount; i++)
            {
                updateProxy(chartArray[i]);
            }
        }

        public void updateProxy(ChartBuildData chart)
        {
            //#pragma message(NV_FILE_LINE "TODO: Use best fit plane instead of average normal.")

            chart.planeNormal = Face.normalizeSafe(chart.normalSum, Vector3.zero, 0.0f);
            chart.centroid = chart.centroidSum / (float)(chart.faces.Count);

            //#pragma message(NV_FILE_LINE "TODO: Experiment with conic fitting.")

            // F = (Nc*Nt - cos Oc)^2 = (x*Nt_x + y*Nt_y + z*Nt_z - cos w)^2
            // dF/dx = 2 * x * (x*Nt_x + y*Nt_y + z*Nt_z - cos w)
            // dF/dy = 2 * y * (x*Nt_x + y*Nt_y + z*Nt_z - cos w)
            // dF/dz = 2 * z * (x*Nt_x + y*Nt_y + z*Nt_z - cos w)
            // dF/dw = 2 * sin w * (x*Nt_x + y*Nt_y + z*Nt_z - cos w)

            // JacobianMatrix({
            // 2 * x * (x*Nt_x + y*Nt_y + z*Nt_z - Cos(w)),
            // 2 * y * (x*Nt_x + y*Nt_y + z*Nt_z - Cos(w)),
            // 2 * z * (x*Nt_x + y*Nt_y + z*Nt_z - Cos(w)),
            // 2 * Sin(w) * (x*Nt_x + y*Nt_y + z*Nt_z - Cos(w))}, {x,y,z,w})

            // H[0,0] = 2 * x * Nt_x + 2 * (x*Nt_x + y*Nt_y + z*Nt_z - cos(w));
            // H[0,1] = 2 * x * Nt_y;
            // H[0,2] = 2 * x * Nt_z;
            // H[0,3] = 2 * x * sin(w);

            // H[1,0] = 2 * y * Nt_x;
            // H[1,1] = 2 * y * Nt_y + 2 * (x*Nt_x + y*Nt_y + z*Nt_z - cos(w));
            // H[1,2] = 2 * y * Nt_z;
            // H[1,3] = 2 * y * sin(w);

            // H[2,0] = 2 * z * Nt_x;
            // H[2,1] = 2 * z * Nt_y;
            // H[2,2] = 2 * z * Nt_z + 2 * (x*Nt_x + y*Nt_y + z*Nt_z - cos(w));
            // H[2,3] = 2 * z * sin(w);

            // H[3,0] = 2 * sin(w) * Nt_x;
            // H[3,1] = 2 * sin(w) * Nt_y;
            // H[3,2] = 2 * sin(w) * Nt_z;
            // H[3,3] = 2 * sin(w) * sin(w) + 2 * cos(w) * (x*Nt_x + y*Nt_y + z*Nt_z - cos(w));

            // @@ Cone fitting might be quite slow.

            /*ConeFitting coneFitting(mesh, 0.1f, 0.001f, 0.001f);

            Vector4 start = Vector4(chart.coneAxis, chart.coneAngle);
            Vector4 solution = coneFitting.solve(chart, start);

            chart.coneAxis = solution.xyz();
            chart.coneAngle = solution.w;*/
        }

        public bool relocateSeeds()
        {
            bool anySeedChanged = false;

            var chartCount = chartArray.Count;
            for (uint i = 0; i < chartCount; i++)
            {
                if (relocateSeed(chartArray[(int)i]))
                {
                    anySeedChanged = true;
                }
            }

            return anySeedChanged;
        }
        
        // Average of the edge midpoints weighted by the edge length.
        // I want a point inside the triangle, but closer to the cirumcenter.
        static Vector3 triangleCenter(Face face)
        {
            Vector3 p0 = face.edge.vertex.pos;
            Vector3 p1 = face.edge.next.vertex.pos;
            Vector3 p2 = face.edge.next.next.vertex.pos;

            float l0 = (p1 - p0).magnitude;
            float l1 = (p2 - p1).magnitude;
            float l2 = (p0 - p2).magnitude;

            Vector3 m0 = (p0 + p1) * l0 / (l0 + l1 + l2);
            Vector3 m1 = (p1 + p2) * l1 / (l0 + l1 + l2);
            Vector3 m2 = (p2 + p0) * l2 / (l0 + l1 + l2);

            return m0 + m1 + m2;
        }

        public bool relocateSeed(ChartBuildData chart)
        {
            Vector3 centroid = computeChartCentroid(chart);

            const uint N = 10;  // @@ Hardcoded to 10?
            var bestTriangles = new PriorityQueue(N);

            // Find the first N triangles that fit the proxy best.
            var faceCount = chart.faces.Count;
            for (var i = 0; i < faceCount; i++)
            {
                float priority = evaluateProxyFitMetric(chart, chart.faces[i]);
                bestTriangles.push(priority, chart.faces[i]);
            }

            // Of those, choose the most central triangle.
            uint mostCentral = 0;
            float maxDistance = -1;

            var bestCount = bestTriangles.Count;
            for (var i = 0; i < bestCount; i++)
            {
                var face = mesh.faceAt((int)bestTriangles.pairs[i].face);
                Vector3 faceCentroid = triangleCenter(face);

                float distance = (centroid - faceCentroid).magnitude;

                /*#pragma message(NV_FILE_LINE "TODO: Implement evaluateDistanceToBoundary.")
                float distance = evaluateDistanceToBoundary(chart, bestTriangles.pairs[i].face);*/

                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    mostCentral = bestTriangles.pairs[i].face;
                }
            }
            Debug.Assert(maxDistance >= 0);

            // In order to prevent k-means cyles we record all the previously chosen seeds.
            var index = chart.seeds.IndexOf(mostCentral);
            if (index != -1)
            {
                // Move new seed to the end of the seed array.
                var last = chart.seeds.Count - 1;
                var seed1 = chart.seeds[index];
                var seed2 = chart.seeds[last];
                chart.seeds[index] = seed2;
                chart.seeds[last] = seed1;
                return false;
            } else
            {
                // Append new seed.
                chart.seeds.Add(mostCentral);
                return true;
            }
        }

        public void updatePriorities(ChartBuildData chart)
        {
            // Re-evaluate candidate priorities.
            var candidateCount = chart.candidates.Count;
            for (var i = 0; i < candidateCount; i++)
            {
                var org = chart.candidates.pairs[i];
                org.priority = evaluatePriority(chart, chart.candidates.pairs[i].face);
                chart.candidates.pairs[i] = org;

                if (faceChartArray[(int)chart.candidates.pairs[i].face] == -1)
                {
                    updateCandidate(chart, chart.candidates.pairs[i].face, chart.candidates.pairs[i].priority);
                }
            }

            // Sort candidates.
            chart.candidates.sort();
        }

        public float evaluatePriority(ChartBuildData chart, uint face)
        {
            // Estimate boundary length and area:
            float newBoundaryLength = evaluateBoundaryLength(chart, face);
            float newChartArea = evaluateChartArea(chart, face);

            float F = evaluateProxyFitMetric(chart, face);
            float C = evaluateRoundnessMetric(chart, face, newBoundaryLength, newChartArea);
            float P = evaluateStraightnessMetric(chart, face);

            // Penalize faces that cross seams, reward faces that close seams or reach boundaries.
            float N = evaluateNormalSeamMetric(chart, face);
            float T = evaluateTextureSeamMetric(chart, face);

            //float R = evaluateCompletenessMetric(chart, face);

            //float D = evaluateDihedralAngleMetric(chart, face);
            // @@ Add a metric based on local dihedral angle.

            // @@ Tweaking the normal and texture seam metrics.
            // - Cause more impedance. Never cross 90 degree edges.
            // - 

            float cost = (float)(
                settings.proxyFitMetricWeight * F +
                settings.roundnessMetricWeight * C +
                settings.straightnessMetricWeight * P +
                settings.normalSeamMetricWeight * N +
                settings.textureSeamMetricWeight * T);

            /*cost = settings.proxyFitMetricWeight * powf(F, settings.proxyFitMetricExponent);
            cost = max(cost, settings.roundnessMetricWeight * powf(C, settings.roundnessMetricExponent));
            cost = max(cost, settings.straightnessMetricWeight * pow(P, settings.straightnessMetricExponent));
            cost = max(cost, settings.normalSeamMetricWeight * N);
            cost = max(cost, settings.textureSeamMetricWeight * T);*/

            // Enforce limits strictly:
            if (newChartArea > settings.maxChartArea) cost = float.MaxValue;
            if (newBoundaryLength > settings.maxBoundaryLength) cost = float.MaxValue;

            // Make sure normal seams are fully respected:
            if (settings.normalSeamMetricWeight >= 1000 && N != 0) cost = float.MaxValue;

            Debug.Log(AtlasPacker.isFinite(cost));
            return cost;
        }
        
        // Unnormalized face normal assuming it's a triangle.
        static Vector3 triangleNormal(Face face)
        {
            Vector3 p0 = face.edge.vertex.pos;
            Vector3 p1 = face.edge.next.vertex.pos;
            Vector3 p2 = face.edge.next.next.vertex.pos;

            Vector3 e0 = p2 - p0;
            Vector3 e1 = p1 - p0;

            return Face.normalizeSafe(Vector3.Cross(e0, e1), Vector3.zero, 0.0f);
        }

        static Vector3 triangleNormalAreaScaled(Face face)
        {
            Vector3 p0 = face.edge.vertex.pos;
            Vector3 p1 = face.edge.next.vertex.pos;
            Vector3 p2 = face.edge.next.next.vertex.pos;

            Vector3 e0 = p2 - p0;
            Vector3 e1 = p1 - p0;

            return Vector3.Cross(e0, e1);
        }

        public float evaluateProxyFitMetric(ChartBuildData chart, uint f)
        {
            var face = mesh.faceAt((int)f);
            Vector3 faceNormal = triangleNormal(face);
            //return AtlasPacker.square(Vector3.Dot(chart.coneAxis, faceNormal) - cosf(chart.coneAngle));

            // Use plane fitting metric for now:
            //return AtlasPacker.square(1 - Vector3.Dot(faceNormal, chart.planeNormal)); // @@ normal deviations should be weighted by face area
            return 1 - Vector3.Dot(faceNormal, chart.planeNormal); // @@ normal deviations should be weighted by face area

            // Find distance to chart.
            /*Vector3 faceCentroid = face.centroid();

            float dist = 0;
            int count = 0;

            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();

                if (!edge.isBoundary()) {
                    var neighborFace = edge.pair().face();
                    if (faceChartArray[neighborFace.id()] == chart.id) {
                        dist += length(neighborFace.centroid() - faceCentroid);
                        count++;
                    }
                }
            }

            dist /= (count * count);

            return (1 - Vector3.Dot(faceNormal, chart.planeNormal)) * dist;*/

            //return (1 - Vector3.Dot(faceNormal, chart.planeNormal));
        }

        public float evaluateDistanceToBoundary(ChartBuildData chart, uint face)
        {
            //#pragma message(NV_FILE_LINE "TODO: Evaluate distance to boundary metric.")

            // @@ This is needed for the seed relocation code.
            // @@ This could provide a better roundness metric.

            return 0.0f;
        }

        public float evaluateDistanceToSeed(ChartBuildData chart, uint f)
        {
            //var seed = chart.seeds.back();
            //var faceCount = mesh.faceCount();
            //return shortestPaths[seed * faceCount + f];

            var seed = mesh.faceAt((int)chart.seeds[chart.seeds.Count - 1]);
            var face = mesh.faceAt((int)f);
            return (triangleCenter(seed) - triangleCenter(face)).magnitude;
        }

        public float evaluateRoundnessMetric(ChartBuildData chart, uint face, float newBoundaryLength, float newChartArea)
        {
            // @@ D-charts use distance to seed.
            // C(c,t) = pi * D(S_c,t)^2 / A_c
            //return PI * AtlasPacker.square(evaluateDistanceToSeed(chart, face)) / chart.area;
            //return PI * AtlasPacker.square(evaluateDistanceToSeed(chart, face)) / chart.area;
            //return 2 * PI * evaluateDistanceToSeed(chart, face) / chart.boundaryLength;

            // Garland's Hierarchical Face Clustering paper uses ratio between boundary and area, which is easier to compute and might work as well:
            // roundness = D^2/4*pi*A . circle = 1, non circle greater than 1

            //return AtlasPacker.square(newBoundaryLength) / (newChartArea * 4 * PI);
            float roundness = AtlasPacker.square(chart.boundaryLength) / chart.area;
            float newRoundness = AtlasPacker.square(newBoundaryLength) / newChartArea;
            if (newRoundness > roundness)
            {
                return AtlasPacker.square(newBoundaryLength) / (newChartArea * 4 * Mathf.PI);
            } else
            {
                // Offer no impedance to faces that improve roundness.
                return 0;
            }

            //return AtlasPacker.square(newBoundaryLength) / (4 * PI * newChartArea);
            //return Mathf.Clamp(1 - (4 * PI * newChartArea) / AtlasPacker.square(newBoundaryLength), 0.0f, 1.0f);

            // Use the ratio between the new roundness vs. the previous roundness.
            // - If we use the absolute metric, when the initial face is very long, then it's hard to make any progress.
            //return (AtlasPacker.square(newBoundaryLength) * chart.area) / (AtlasPacker.square(chart.boundaryLength) * newChartArea);
            //return (4 * PI * newChartArea) / AtlasPacker.square(newBoundaryLength) - (4 * PI * chart.area) / AtlasPacker.square(chart.boundaryLength);

            //if (AtlasPacker.square(newBoundaryLength) * chart.area) / (AtlasPacker.square(chart.boundaryLength) * newChartArea);

        }

        public float evaluateStraightnessMetric(ChartBuildData chart, uint f)
        {
            float l_out = 0.0f;
            float l_in = 0.0f;

            var face = mesh.faceAt((int)f);
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();

                //float l = edge.length();
                float l = edgeLengths[(int)edge.id / 2];

                if (edge.isBoundary())
                {
                    l_out += l;
                } else
                {
                    var neighborFaceId = edge.pair.face.id;
                    if (faceChartArray[(int)neighborFaceId] != chart.id)
                    {
                        l_out += l;
                    } else
                    {
                        l_in += l;
                    }
                }
            }
            Debug.Assert(l_in != 0.0f); // Candidate face must be adjacent to chart. @@ This is not true if the input mesh has zero-length edges.

            //return l_out / l_in;
            float ratio = (l_out - l_in) / (l_out + l_in);
            //if (ratio < 0) ratio *= 10; // Encourage closing gaps.
            return Math.Min(ratio, 0.0f); // Only use the straightness metric to close gaps.
                                          //return ratio;
        }

        

        static bool isNormalSeam(Edge edge) 
        {
            return (edge.vertex.nor != edge.pair.next.vertex.nor || edge.next.vertex.nor != edge.pair.vertex.nor);
        }

        static bool isTextureSeam(Edge edge) 
        {
            return (edge.vertex.tex != edge.pair.next.vertex.tex || edge.next.vertex.tex != edge.pair.vertex.tex);
        }

        public float evaluateNormalSeamMetric(ChartBuildData chart, uint f)
        {
            float seamFactor = 0.0f;
            float totalLength = 0.0f;

            var face = mesh.faceAt((int)f);
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();

                if (edge.isBoundary())
                {
                    continue;
                }

                var neighborFaceId = edge.pair.face.id;
                if (faceChartArray[(int)neighborFaceId] != chart.id)
                {
                    continue;
                }

                //float l = edge.length();
                float l = edgeLengths[(int)edge.id / 2];

                totalLength += l;

                if (!edge.isSeam())
                {
                    continue;
                }

                // Make sure it's a normal seam.
                if (isNormalSeam(edge))
                {
                    float d0 = Mathf.Clamp(Vector3.Dot(edge.vertex.nor, edge.pair.next.vertex.nor), 0.0f, 1.0f);
                    float d1 = Mathf.Clamp(Vector3.Dot(edge.next.vertex.nor, edge.pair.vertex.nor), 0.0f, 1.0f);
                    //float a0 = Mathf.Clamp(acosf(d0) / (PI/2), 0.0f, 1.0f);
                    //float a1 = Mathf.Clamp(acosf(d1) / (PI/2), 0.0f, 1.0f);
                    //l *= (a0 + a1) * 0.5f;

                    l *= 1 - (d0 + d1) * 0.5f;

                    seamFactor += l;
                }
            }

            if (seamFactor == 0) return 0.0f;
            return seamFactor / totalLength;
        }

        public float evaluateTextureSeamMetric(ChartBuildData chart, uint f)
        {
            float seamLength = 0.0f;
            //float newSeamLength = 0.0f;
            //float oldSeamLength = 0.0f;
            float totalLength = 0.0f;

            var face = mesh.faceAt((int)f);
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();

                /*float l = edge.length();
                totalLength += l;

                if (edge.isBoundary() || !edge.isSeam()) {
                    continue;
                }

                // Make sure it's a texture seam.
                if (isTextureSeam(edge))
                {
                    uint neighborFaceId = edge.pair().face().id();
                    if (faceChartArray[neighborFaceId] != chart.id) {
                        newSeamLength += l;
                    }
                    else {
                        oldSeamLength += l;
                    }
                }*/

                if (edge.isBoundary())
                {
                    continue;
                }

                var neighborFaceId = edge.pair.face.id;
                if (faceChartArray[(int)neighborFaceId] != chart.id)
                {
                    continue;
                }

                //float l = edge.length();
                float l = edgeLengths[(int)edge.id / 2];
                totalLength += l;

                if (!edge.isSeam())
                {
                    continue;
                }

                // Make sure it's a texture seam.
                if (isTextureSeam(edge))
                {
                    seamLength += l;
                }
            }

            if (seamLength == 0.0f)
            {
                return 0.0f; // Avoid division by zero.
            }

            return seamLength / totalLength;
        }

        public float evaluateSeamMetric(ChartBuildData chart, uint f)
        {
            float newSeamLength = 0.0f;
            float oldSeamLength = 0.0f;
            float totalLength = 0.0f;

            var face = mesh.faceAt((int)f);
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();

                //float l = edge.length();
                float l = edgeLengths[(int)edge.id / 2];

                if (edge.isBoundary())
                {
                    newSeamLength += l;
                } else
                {
                    if (edge.isSeam())
                    {
                        uint neighborFaceId = edge.pair.face.id;
                        if (faceChartArray[(int)neighborFaceId] != chart.id)
                        {
                            newSeamLength += l;
                        } else
                        {
                            oldSeamLength += l;
                        }
                    }
                }

                totalLength += l;
            }

            return (newSeamLength - oldSeamLength) / totalLength;
        }

        public float evaluateChartArea(ChartBuildData chart, uint f)
        {
            var face = mesh.faceAt((int)f);
            //return chart.area + face.area();
            return chart.area + faceAreas[(int)face.id];
        }

        public float evaluateBoundaryLength(ChartBuildData chart, uint f)
        {
            float boundaryLength = chart.boundaryLength;

            // Add new edges, subtract edges shared with the chart.
            var face = mesh.faceAt((int)f);
            for (var it = face.edges(); !it.isDone(); it.advance())
            {
                var edge = it.current();
                //float edgeLength = edge.length();
                float edgeLength = edgeLengths[(int)edge.id / 2];

                if (edge.isBoundary())
                {
                    boundaryLength += edgeLength;
                } else
                {
                    uint neighborFaceId = edge.pair.face.id;
                    if (faceChartArray[(int)neighborFaceId] != chart.id)
                    {
                        boundaryLength += edgeLength;
                    } else
                    {
                        boundaryLength -= edgeLength;
                    }
                }
            }
            //Debug.Assert(boundaryLength >= 0);

            return Math.Max(0.0f, boundaryLength);  // @@ Hack!
        }

        public Vector3 evaluateChartNormalSum(ChartBuildData chart, uint f)
        {
            var face = mesh.faceAt((int)f);
            return chart.normalSum + triangleNormalAreaScaled(face);
        }


        public Vector3 evaluateChartCentroidSum(ChartBuildData chart, uint f)
        {
            var face = mesh.faceAt((int)f);
            return chart.centroidSum + face.centroid();
        }

        public Vector3 computeChartCentroid(ChartBuildData chart)
        {
            Vector3 centroid = Vector3.zero;

            var faceCount = chart.faces.Count;
            for (var i = 0; i < faceCount; i++)
            {
                var face = mesh.faceAt((int)chart.faces[i]);
                centroid += triangleCenter(face);
            }

            return centroid / (float)(faceCount);
        }



        public void fillHoles(float threshold)
        {
            while (facesLeft > 0)
            {
                createRandomChart(threshold);
            }
        }

        public void mergeCharts()
        {
            var chartCount = chartArray.Count;
            for (int c = chartCount - 1; c >= 0; c--)
            {
                var sharedBoundaryLengths = new float[chartCount];

                var chart = chartArray[c];

                float externalBoundary = 0.0f;

                var faceCount = chart.faces.Count;
                for (var i = 0; i < faceCount; i++)
                {
                    var f = chart.faces[i];
                    var face = mesh.faceAt((int)f);

                    for (var it = face.edges(); !it.isDone(); it.advance())
                    {
                        var edge = it.current();

                        //float l = edge.length();
                        float l = edgeLengths[(int)edge.id / 2];

                        if (edge.isBoundary())
                        {
                            externalBoundary += l;
                        } else
                        {
                            var neighborFace = edge.pair.face.id;
                            var neighborChart = faceChartArray[(int)neighborFace];

                            if (neighborChart != c)
                            {
                                if ((edge.isSeam() && (isNormalSeam(edge) || isTextureSeam(edge))) || neighborChart == -2)
                                {
                                    externalBoundary += l;
                                } else
                                {
                                    sharedBoundaryLengths[(int)neighborChart] += l;
                                }
                            }
                        }
                    }
                }

                for (int cc = chartCount - 1; cc >= 0; cc--)
                {
                    if (cc == c)
                        continue;

                    var chart2 = chartArray[cc];
                    if (chart2 == null)
                        continue;

                    if (sharedBoundaryLengths[cc] > 0.8 * Math.Max(0.0f, chart.boundaryLength - externalBoundary))
                    {

                        // Try to avoid degenerate configurations.
                        if (chart2.boundaryLength > sharedBoundaryLengths[cc])
                        {
                            if (Vector3.Dot(chart2.planeNormal, chart.planeNormal) > -0.25)
                            {
                                mergeChart(chart2, chart, sharedBoundaryLengths[cc]);
                                //delete chart;
                                chartArray[c] = null;
                                break;
                            }
                        }
                    }

                    if (sharedBoundaryLengths[cc] > 0.20 * Math.Max(0.0f, chart.boundaryLength - externalBoundary))
                    {

                        // Compare proxies.
                        if (Vector3.Dot(chart2.planeNormal, chart.planeNormal) > 0)
                        {
                            mergeChart(chart2, chart, sharedBoundaryLengths[cc]);
                            //delete chart;
                            chartArray[c] = null;
                            break;
                        }
                    }
                }
            }

            // Remove deleted charts.
            for (var  c = 0; c < (int)(chartArray.Count); /*do not increment if removed*/)
            {
                if (chartArray[c] == null)
                {
                    chartArray.RemoveAt(c);

                    // Update faceChartArray.
                    var faceCount = faceChartArray.Length;
                    for (var i = 0; i < faceCount; i++)
                    {
                        Debug.Assert(faceChartArray[i] != -1);
                        Debug.Assert(faceChartArray[i] != c);
                        Debug.Assert(faceChartArray[i] <= (int)(chartArray.Count));

                        if (faceChartArray[i] > c)
                        {
                            faceChartArray[i]--;
                        }
                    }
                } else
                {
                    chartArray[c].id = c;
                    c++;
                }
            }
        }

        // @@ Cleanup.
        public struct Candidate {
            public uint face;
            public ChartBuildData chart;
            public float metric;
        };

        public Candidate getBestCandidate()
        {
            uint best = 0;
            float bestCandidateMetric = float.MaxValue;

            var candidateCount = candidateArray.Count;
            Debug.Log(candidateCount > 0);

            for (uint i = 0; i < candidateCount; i++)
            {
                var candidate = candidateArray[(int)i];

                if (candidate.metric < bestCandidateMetric)
                {
                    bestCandidateMetric = candidate.metric;
                    best = i;
                }
            }

            return candidateArray[(int)best];
        }


        public void removeCandidate(uint f)
        {
            int c = (int)faceCandidateArray[(int)f];
            if (c != -1)
            {
                faceCandidateArray[(int)f] = unchecked((uint)-1);

                if (c == candidateArray.Count - 1)
                {
                    candidateArray.RemoveAt(candidateArray.Count - 1);
                } else
                {
                    candidateArray[c] = candidateArray[candidateArray.Count - 1];
                    candidateArray.RemoveAt(candidateArray.Count - 1);
                    faceCandidateArray[(int)candidateArray[c].face] = (uint)c;
                }
            }
        }

        public void updateCandidate(ChartBuildData chart, uint f, float metric)
        {
            if (faceCandidateArray[(int)f] == unchecked((uint)-1))
            {
                var index = candidateArray.Count;
                faceCandidateArray[(int)f] = (uint)index;
                candidateArray.Add(new Candidate
                {
                    face = f,
                    chart = chart,
                    metric = metric
                });
            } else
            {
                int c = (int)faceCandidateArray[(int)f];
                Debug.Assert(c != -1);

                var candidate = candidateArray[c];
                Debug.Assert(candidate.face == f);

                if (metric < candidate.metric || chart == candidate.chart)
                {
                    candidate.metric = metric;
                    candidate.chart = chart;
                }
            }

        }

        public void mergeChart(ChartBuildData owner, ChartBuildData chart, float sharedBoundaryLength)
        {
            var faceCount = chart.faces.Count;
            for (var i = 0; i < faceCount; i++)
            {
                var f = chart.faces[i];

                Debug.Assert(faceChartArray[(int)f] == chart.id);
                faceChartArray[(int)f] = owner.id;

                owner.faces.Add(f);
            }

            // Update adjacencies?

            owner.area += chart.area;
            owner.boundaryLength += chart.boundaryLength - sharedBoundaryLength;

            owner.normalSum += chart.normalSum;
            owner.centroidSum += chart.centroidSum;

            updateProxy(owner);
        }


        public uint chartCount() { return (uint)chartArray.Count; }
        public List<uint> chartFaces(uint i)
        {
            return chartArray[(int)i].faces;
        }

        public ChartMesh mesh;
        public uint facesLeft;
        public int[] faceChartArray;
        public List<ChartBuildData> chartArray = new List<ChartBuildData>();

        public float[] edgeLengths;
        public float[] faceAreas;

        public List<Candidate> candidateArray = new List<Candidate>(); //
        public uint[] faceCandidateArray; // Map face index to candidate index.

        public SegmentationSettings settings;
    };



    class ChartBuildData
    {
        public ChartBuildData(int id) 
        {
            this.id = id;
            planeNormal = Vector3.zero;
            centroid = Vector3.zero;
            coneAxis = Vector3.zero;
            coneAngle = 0;
            area = 0;
            boundaryLength = 0;
            normalSum = Vector3.zero;
            centroidSum = Vector3.zero;
        }

        public int id;

        // Proxy info:
        public Vector3 planeNormal;
        public Vector3 centroid;
        public Vector3 coneAxis;
        public float coneAngle;

        public float area;
        public float boundaryLength;
        public Vector3 normalSum;
        public Vector3 centroidSum;

        public List<uint> seeds = new List<uint>();  // @@ These could be a pointers to the HalfEdge faces directly.
        public List<uint> faces = new List<uint>();
        public PriorityQueue candidates = new PriorityQueue();
    };
    

    // Dummy implementation of a priority queue using sort at insertion.
    // - Insertion is o(n)
    // - Smallest element goes at the end, so that popping it is o(1).
    // - Resorting is n*log(n)
    // @@ Number of elements in the queue is usually small, and we'd have to rebalance often. I'm not sure it's worth implementing a heap.
    // @@ Searcing at removal would remove the need for sorting when priorities change.
    class PriorityQueue
    {
        public PriorityQueue(uint size = uint.MaxValue) { maxSize = size; }

        public void push(float priority, uint face) 
        {
            var i = 0;
            var count = pairs.Count;
            for (; i < count; i++) {
                if (pairs[i].priority > priority) break;
            }

            Pair p = new Pair { priority = priority, face = face };
            pairs.Insert(i, p);

            if (pairs.Count > maxSize) {
                pairs.RemoveAt(0);
            }
        }

        // push face out of order, to be sorted later.
        public void push(uint face) {
            Pair p = new Pair { priority = 0.0f, face = face };
            pairs.Add(p);
        }

        public uint pop() {
            uint f = pairs[pairs.Count - 1].face;
            pairs.RemoveAt(pairs.Count - 1);
            return f;
        }

        public void sort() 
        {
            pairs.Sort(delegate (Pair x, Pair y)
            {
                if (x.priority == y.priority)
                    return 0;
                // !! Sort in inverse priority order!
                if (x.priority < y.priority)
                    return 1;
                return -1;
            });
            //nv::sort(pairs); // @@ My intro sort appears to be much slower than it should!
            //std::sort(pairs.buffer(), pairs.buffer() + pairs.count());
        }

        public void Clear() {
            pairs.Clear();
        }

        public int Count { get { return pairs.Count; } }

        public float firstPriority() { return pairs[pairs.Count - 1].priority; }


        uint maxSize;
        
        public struct Pair
        {
            public float priority;
            public uint face;
        }
        

        public List<Pair> pairs = new List<Pair>();
    };
}