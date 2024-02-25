using System.Collections.Generic;
using UnityEngine;
using Utils;

public class Grid : MonoBehaviour
{
    //  Модель для отрисовки узла сетки
    public GameObject nodeModel;

    //  Ландшафт (Terrain) на котором строится путь
    [SerializeField] private Terrain landscape = null;

    //  Шаг сетки (по x и z) для построения точек
    [SerializeField] private int gridDelta = 20;

    //  Номер кадра, на котором будет выполнено обновление путей
    private int updateAtFrame = 0;

    //  Массив узлов - создаётся один раз, при первом вызове скрипта
    private PathNode[,] grid = null;

    private void CheckWalkableNodes()
    {
        foreach (PathNode node in grid)
        {
            /*
            //  Пока что считаем все вершины проходимыми, без учёта препятствий
            node.walkable = true;
            */
            node.walkable = !Physics.CheckSphere(node.body.transform.position, 1);
            if (node.walkable) node.Illuminate();
            else node.Fade();
        }
    }


    private void InitPathNodes()
    {
        //  Создаём сетку узлов для навигации - адаптивную, под размер ландшафта
        Vector3 terrainSize = landscape.terrainData.bounds.size;
        int sizeX = (int)(terrainSize.x / gridDelta);
        int sizeZ = (int)(terrainSize.z / gridDelta);
        //  Создаём и заполняем сетку вершин, приподнимая на 25 единиц над ландшафтом
        grid = new PathNode[sizeX, sizeZ];
        for (int x = 0; x < sizeX; ++x)
        for (int z = 0; z < sizeZ; ++z)
        {
            Vector3 position = new Vector3(x * gridDelta, 0, z * gridDelta);
            position.y = landscape.SampleHeight(position) + 25;
            grid[x, z] = new PathNode(nodeModel, false, position);
            grid[x, z].ParentNode = null;
            grid[x, z].Fade();
        }
    }
    
    private void InitBoxes(
        int boxesCount,
        Vector2 widthBounds ,
        Vector2 heightBounds,
        Vector2 depthBounds 
        )
    {
        Vector3 terrainSize = landscape.terrainData.bounds.size;
        Vector3 terrainPosition =  landscape.transform.position;
        
        for (int i = 0; i < boxesCount; i++)
        {
            // Generate random position within the terrain bounds
            Vector3 position = new Vector3(
                Random.Range(terrainPosition.x, terrainPosition.x + terrainSize.x),
                0, 
                Random.Range(terrainPosition.z, terrainPosition.z + terrainSize.z)
            );
            position.y = landscape.SampleHeight(position) + 100;
            
            // Create the box at the random position with random size
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.transform.position = position;
            box.transform.localScale = new Vector3(
                Random.Range(widthBounds.x, widthBounds.y), 
                Random.Range(heightBounds.x, heightBounds.y), 
                Random.Range(depthBounds.x, depthBounds.y)
            );
            box.AddComponent<BoxCollider>();
            Rigidbody rb = box.AddComponent<Rigidbody>();
            rb.useGravity = true;
        }
    }
    
    // Метод вызывается однократно перед отрисовкой первого кадра
    void Start()
    {
        InitPathNodes();
        InitBoxes(
            10,
            new Vector2(50f, 150f),
            new Vector2(20f, 60f),
            new Vector2(50f, 150f)
            );
    }

    /// <summary>
    /// Получение списка соседних узлов для вершины сетки
    /// </summary>
    /// <param name="current">индексы текущей вершины </param>
    /// <returns></returns>
    private List<Vector2Int> GetNeighbours(Vector2Int current)
    {
        List<Vector2Int> nodes = new List<Vector2Int>();
        for (int x = current.x - 1; x <= current.x + 1; ++x)
        for (int y = current.y - 1; y <= current.y + 1; ++y)
            if (
                x >= 0
                && y >= 0
                && x < grid.GetLength(0)
                && y < grid.GetLength(1)
                && (x != current.x || y != current.y)
            )
                nodes.Add(new Vector2Int(x, y));
        return nodes;
    }

    private void SetupGrid()
    {
        //  Очищаем все узлы - сбрасываем отметку родителя, снимаем подсветку
        foreach (var node in grid)
        {
            node.Fade();
            node.ParentNode = null;
        }
        
        //  На данный момент вызов этого метода не нужен, там только устанавливается проходимость вершины. Можно добавить обработку препятствий
        CheckWalkableNodes();
    }

    /// <summary>
    /// Вычисление "кратчайшего" между двумя вершинами сетки
    /// </summary>
    /// <param name="startNode">Координаты начального узла пути (индексы элемента в массиве grid)</param>
    /// <param name="finishNode">Координаты конечного узла пути (индексы элемента в массиве grid)</param>
    void SamplePathfinder(Vector2Int startNode, Vector2Int finishNode)
    {
        SetupGrid();

        //  Реализуется аналог волнового алгоритма, причём найденный путь не будет являться оптимальным 

        PathNode start = grid[startNode.x, startNode.y];

        //  Начальную вершину отдельно изменяем
        start.ParentNode = null;
        start.DistanceToStart = 0;

        //  Очередь вершин в обработке - в A* необходимо заменить на очередь с приоритетом
        Queue<Vector2Int> nodes = new Queue<Vector2Int>();
        //  Начальную вершину помещаем в очередь
        nodes.Enqueue(startNode);
        //  Пока не обработаны все вершины (очередь содержит узлы для обработки)
        while (nodes.Count != 0)
        {
            Vector2Int current = nodes.Dequeue();
            
            //  Если достали целевую - можно заканчивать (это верно и для A*)
            if (current == finishNode) break;
            
            var currentNode = grid[current.x, current.y];
            
            //  Получаем список соседей
            var neighbours = GetNeighbours(current);
            foreach (var node in neighbours)
            {
                var neighbor = grid[node.x, node.y];
                
                if (!neighbor.walkable) continue;
                
                if (neighbor.DistanceToStart > currentNode.DistanceToStart + PathNode.HeightPenaltyDist(neighbor, currentNode))
                {
                    neighbor.ParentNode = currentNode;
                    nodes.Enqueue(node);
                }
            }
        }

        HighlightPath(finishNode);
    }

    public delegate float DistanceMetric(PathNode from, PathNode to);
    
    void AStarPathfinder(
        Vector2Int start,
        Vector2Int finish,
        DistanceMetric getDistance,
        DistanceMetric getHScore
    )
    {
        SetupGrid();
        
        // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        Dictionary<PathNode, float> gScore = new Dictionary<PathNode, float>();
        foreach (var node in grid)
        {
            gScore[node] = float.PositiveInfinity;
        }
        
        // The queue of discovered nodes that need to be expanded.
        PriorityQueue<Vector2Int, float> openNodes = new PriorityQueue<Vector2Int, float>();
        
        // already processed nodes
        HashSet<Vector2Int> visitedNodes = new HashSet<Vector2Int>();
        
        PathNode startNode = grid[start.x, start.y];
        PathNode finishNode = grid[finish.x, finish.y];

        //  Начальную вершину отдельно изменяем
        startNode.ParentNode = null;
        gScore[startNode] = 0;
        
        openNodes.Enqueue(start, 0);
        
        while (openNodes.Count > 0)
        {
            Vector2Int currentGridNode = openNodes.Dequeue();

            //  Если достали целевую - можно заканчивать (это верно и для A*)
            if (currentGridNode == finish) break;
            
            visitedNodes.Add(currentGridNode);
            
            PathNode currentNode = grid[currentGridNode.x, currentGridNode.y];

            var neighbours = GetNeighbours(currentGridNode);
            foreach (var neighborGridNode in neighbours)
            {
                var neighbor = grid[neighborGridNode.x, neighborGridNode.y];

                if (!neighbor.walkable || visitedNodes.Contains(neighborGridNode)) continue;

                // d(current,neighbor) is the weight of the edge from current to neighbor
                var d = getDistance(neighbor, currentNode);
                
                // g(n) distance from start to the neighbor through current
                var tentativeGScore = gScore[currentNode] + d;
                
                // KeyNotFoundException: The given key 'PathNode' was not present in the dictionary.
                if (tentativeGScore >= gScore[neighbor]) continue;
                // This path to neighbor is better than any previous one. Record it!
                
                neighbor.ParentNode = currentNode;
                gScore[neighbor] = tentativeGScore;

                // h(n) is estimated cost of the cheapest path from n to the goal
                var h = getHScore(neighbor, finishNode);
                
                // f(n) = g(n) + h(n)
                var fScore = tentativeGScore + h;
                openNodes.Enqueue(neighborGridNode, fScore);
            }
        }

        HighlightPath(finish);
    }

    void DijkstraPathfinder(
        Vector2Int start,
        Vector2Int finish,
        DistanceMetric getDistance
    )
    {
        AStarPathfinder(
            start,
            finish,
            getDistance,
            (x, y) => 0
        );
    }

    private void HighlightPath(Vector2Int finishNode)
    {
        //  Восстанавливаем путь от целевой к стартовой
        var pathElem = grid[finishNode.x, finishNode.y];
        while (pathElem != null)
        {
            pathElem.Highlight();
            pathElem = pathElem.ParentNode;
        }
    }

    // Метод вызывается каждый кадр
    void Update()
    {
        //  Чтобы не вызывать этот метод каждый кадр, устанавливаем интервал вызова в 1000 кадров
        if (Time.frameCount < updateAtFrame) return;
        updateAtFrame = Time.frameCount + 1000;

        // SamplePathfinder(new Vector2Int(0, 0), new Vector2Int(grid.GetLength(0)-1, grid.GetLength(1)-1));
        /*AStarPathfinder(
            new Vector2Int(0, 0), 
            new Vector2Int(grid.GetLength(0) - 1, grid.GetLength(1) - 1),
            PathNode.HeightPenaltyDist,
            PathNode.HeightPenaltyDist
        );*/
        DijkstraPathfinder(
            new Vector2Int(0, 0), 
            new Vector2Int(grid.GetLength(0) - 1, grid.GetLength(1) - 1),
            PathNode.HeightPenaltyDist
        );
    }
}