using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFinding_Report
{
    public class AStar
    {
        const int CostStraight = 10;    // 직선 거리값           // 값을 1로 할경우 다익스트라알고리즘과 같아지기애 10을 기본값으로 선정함
        const int CostDiagonal = 14;    // 대각선 거리값

        static Point[] Direction =
        {
            new Point(0, +1),               // 상
            new Point(0, -1),               // 하
            new Point(-1, 0),               // 좌
            new Point(+1, 0),               // 우
            new Point( -1, +1 ),		    // 좌상
			new Point( -1, -1 ),		    // 좌하
			new Point( +1, +1 ),		    // 우상
			new Point( +1, -1 )		    // 우하
        };

        public static bool PathFinding(bool[,] tileMap, Point start, Point end, out List<Point> path)
        {
            // =======초기화========
            int ySize = tileMap.GetLength(0);           // y, x 둘다 따로 길이를 갖고있을 수 있게 해주어 다양한 맵을 구현하게 함
            int xSize = tileMap.GetLength(0);

            bool[,] visited = new bool[ySize, xSize];   // 방문변수 : 해당 좌표에 방문되었음을 갱신하기 위한 변수
            ASNode[,] nodes = new ASNode[ySize, xSize]; // 정점
            PriorityQueue<ASNode, int> nextPointPQ = new PriorityQueue<ASNode, int>();  // 우선순위큐로 가장낮은값을 가진 정점을 우선순위로 꺼내지도록 사용
            // ====================

            // 0. 시작정점 생성하여 추가
            ASNode startNode = new ASNode(start, null, 0, Heuristic(start, end));
            nodes[startNode.point.y, startNode.point.x] = startNode;
            nextPointPQ.Enqueue(startNode, startNode.f);

            while (nextPointPQ.Count > 0)               // 우선순위큐에 넣어진 정점들이 다 빌때까지 반복(빈상태)
            {
                // 1. 다음으로 탐색할 정점 꺼내기
                ASNode nextNode = nextPointPQ.Dequeue();

                // 2. 방문한 정점은 방문표시
                visited[nextNode.point.y, nextNode.point.x] = true;

                // 3. 다음으로 탐색할 정점이 도착지인 경우
                // 도착했다고 판단해서 경로 반환
                if (nextNode.point.x == end.x && nextNode.point.y == end.y)
                {
                    // 목표부터 역순으로 제작
                    Point? pathPoint = end;
                    path = new List<Point>();

                    while (pathPoint != null)    // 지금 정점의 포인트가 시작지점까지 갈때까지
                    {
                        Point point = pathPoint.GetValueOrDefault();    // null일경우를 생각해서 get를 넣어줌
                        path.Add(point);
                        pathPoint = nodes[point.y, point.x].parent;     // 그다음 정점은 부모정점으로 가게 하여 목표가 역순으로 올라가게 설정
                    }
                    path.Reverse(); // 역순을 다시 정순으로 변환하는 Reverse를 통해 뒤집어 주는 과정이 필요함 
                    return true;
                }

                // 4. AStar 탐색을 진행                  // 찾을때 까지 상하좌우로 탐색
                // 방향 탐색
                for (int i = 0; i < Direction.Length; i++)
                {
                    int x = nextNode.point.x + Direction[i].x;
                    int y = nextNode.point.y + Direction[i].y;

                    // 4-1. 탐색하면 안되는 경우
                    // 맵을 벗어났을 경우
                    if (x < 0 || x >= xSize || y < 0 || y >= ySize) continue;  // 맵범위 이외 위치일경우 건너뛰기

                    // 탐색할 수 없는 정점일 경우
                    else if (tileMap[y, x] == false) continue;

                    // 이미 방문한 정점일 경우
                    else if (visited[y, x]) continue;


                    // 4-2. 탐색할 정점 만들기 (점수계산)
                    int g = nextNode.g + ((nextNode.point.x == x || nextNode.point.y == y) ? CostStraight : CostDiagonal);      // 팔방향에 대해 대각에 대한 코드로 넣어주면 됨
                    // == int g = nextNode.g +10;
                    int h = Heuristic(new Point(x, y), end);    // 현재 정점에서 end까지 거리
                    ASNode newNode = new ASNode(new Point(x, y), nextNode.point, g, h);

                    // 4-3. 정점의 갱신이 필요한 경우 새로운 정점으로 할당
                    if (nodes[y, x] == null ||      // 점수계산이 되지 않은 정점이거나
                        nodes[y, x].f > newNode.f)  // 다음 정점보다 현재정점이 가중치가 더높은 경우
                    {
                        nodes[y, x] = newNode;
                        nextPointPQ.Enqueue(newNode, newNode.f);
                    }
                }
            }
            path = null;        // 경로를 못찾았을 경우
            return false;
        }

        // 휴리스틱 (Heuristic) : 최상의 경로를 추정하는 순위값, 휴리스틱에 의해 경로탑색 효율이 결정됨
        private static int Heuristic(Point start, Point end)
        {
            int xSize = Math.Abs(start.x - end.x);  // 가로로 가야하는 횟수
            int ySize = Math.Abs(start.y - end.y);  // 세로로 가야하는 횟수

            // 맨해튼 거리 : 가로 세로를 통해 이동하는 거리
            // return CostStraight * (xSize + ySize);

            // 유클리드 거리 : 대각선을 통해 이동하는 거리 (직선거리)
            return CostStraight * (int)Math.Sqrt(xSize * xSize + ySize * ySize);
        }


        private class ASNode
        {
            public Point point;     // 현재 정점 위치
            public Point? parent;    // 이 정점을 탐색한 정점, 탐색당한 위치가 없을수도 있기애 ?(nullable)을 해준다


            public int f;           // f(x) = g(x) + h(x) : 총 거리
            public int g;           // 현재까지의 거리, 즉 지금까지 경로 가중치   
            public int h;           // 휴리스틱 : 앞으로 예상되는 거리, 목표까지 추정 경로 가중치

            public ASNode(Point point, Point? parent, int g, int h)
            {
                this.point = point;
                this.parent = parent;
                this.g = g;
                this.h = h;
                this.f = g + h;
            }
        }
    }
    public struct Point
    {
        public int x;
        public int y;

        public Point(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }
}
