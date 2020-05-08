using System.Collections.Generic;
using System.Linq;
using Roy_T.AStar.Collections;
using Roy_T.AStar.Graphs;
using Roy_T.AStar.Grids;
using Roy_T.AStar.Primitives;

namespace Roy_T.AStar.Paths
{
    public sealed class PathFinder
    {
        private readonly MinHeap<PathFinderNode> Interesting;
        private readonly Dictionary<INode, PathFinderNode> Nodes; // Key是图形中的节点值，Value是指经过该点所需的时间，以及走到终点大概还需要的总时间
        private readonly PathReconstructor PathReconstructor;

        private PathFinderNode NodeClosestToGoal;

        public PathFinder()
        {
            this.Interesting = new MinHeap<PathFinderNode>();
            this.Nodes = new Dictionary<INode, PathFinderNode>();
            this.PathReconstructor = new PathReconstructor();
        }

        public Path FindPath(GridPosition start, GridPosition end, Grid grid)
        {
            var startNode = grid.GetNode(start);
            var endNode = grid.GetNode(end);

            var maximumVelocity = grid.GetAllNodes().SelectMany(n => n.Outgoing).Select(e => e.TraversalVelocity).Max();

            return this.FindPath(startNode, endNode, maximumVelocity);
        }

        public Path FindPath(GridPosition start, GridPosition end, Grid grid, Velocity maximumVelocity)
        {
            var startNode = grid.GetNode(start);
            var endNode = grid.GetNode(end);

            return this.FindPath(startNode, endNode, maximumVelocity);
        }

        /// <summary>
        /// 根据起止点以及速度计算路径
        /// </summary>
        /// <param name="start"></param>
        /// <param name="goal"></param>
        /// <param name="maximumVelocity"></param>
        /// <returns></returns>
        public Path FindPath(INode start, INode goal, Velocity maximumVelocity)
        {
            this.ResetState();
            this.AddFirstNode(start, goal, maximumVelocity);

            while (this.Interesting.Count > 0)
            {
                var current = this.Interesting.Extract();
                if (GoalReached(goal, current))
                {
                    return this.PathReconstructor.ConstructPathTo(current.Node, goal);
                }

                this.UpdateNodeClosestToGoal(current);

                foreach (var edge in current.Node.Outgoing)
                {
                    var oppositeNode = edge.End;
                    var costSoFar = current.DurationSoFar + edge.TraversalDuration;

                    // Nodes 记录了目前从起点出发到达的终点之间可能计算的点路径，以及到达这个点的路径的所需时间以及到终点还需时间
                    // 目前这个nodes不会记录所有的网格节点，本身在递归处理的时候相当于做了效率优化。提升了运行了速度
                    if (this.Nodes.TryGetValue(oppositeNode, out var node))
                    {
                        this.UpdateExistingNode(goal, maximumVelocity, current, edge, oppositeNode, costSoFar, node);
                    }
                    else
                    {
                        this.InsertNode(oppositeNode, edge, goal, costSoFar, maximumVelocity);
                    }
                }
            }

            return this.PathReconstructor.ConstructPathTo(this.NodeClosestToGoal.Node, goal);
        }

        private void ResetState()
        {
            this.Interesting.Clear();
            this.Nodes.Clear();
            this.PathReconstructor.Clear();
            this.NodeClosestToGoal = null;
        }

        private void AddFirstNode(INode start, INode goal, Velocity maximumVelocity)
        {
            var head = new PathFinderNode(start, Duration.Zero, ExpectedDuration(start, goal, maximumVelocity));
            this.Interesting.Insert(head);
            this.Nodes.Add(head.Node, head);
            this.NodeClosestToGoal = head;
        }

        private static bool GoalReached(INode goal, PathFinderNode current) => current.Node == goal;

        private void UpdateNodeClosestToGoal(PathFinderNode current)
        {
            if (current.ExpectedRemainingTime < this.NodeClosestToGoal.ExpectedRemainingTime)
            {
                this.NodeClosestToGoal = current;
            }
        }

        /// <summary>
        /// 修改到现有节点所需的最少时间方案
        /// </summary>
        /// <param name="goal"></param>
        /// <param name="maximumVelocity"></param>
        /// <param name="current"></param>
        /// <param name="edge"></param>
        /// <param name="oppositeNode"></param>
        /// <param name="costSoFar"></param>
        /// <param name="node"></param>
        private void UpdateExistingNode(INode goal, Velocity maximumVelocity, PathFinderNode current, IEdge edge, INode oppositeNode, Duration costSoFar, PathFinderNode node)
        {
            if (node.DurationSoFar > costSoFar)
            {
                this.Interesting.Remove(node);// 删除到达该点原有的路线方案，PathFinderNode 中记录了到达该点的位置信息，以及到达该点所有的时间，以及走完到终点大概所需的总时间

                // 上面的只是删除可能感兴趣的PathFinderNode， 删除这个pathFinderNode,后面这个InsertNode还会继续增加增加该点信息，
                // 1， 上面的删除其实也只是修改到达该点的信息，
                // 2. InsertNode中的PathReconstructor 中的SetCameFrom中的容器会修改这个路径来源，其实没有修改点Node,而是修改点的value值， Value值其实是路径值
                // 路径值表示出发点是哪一个点，结束点是当前点
                this.InsertNode(oppositeNode, edge, goal, costSoFar, maximumVelocity);
            }
        }

        private void InsertNode(INode current, IEdge via, INode goal, Duration costSoFar, Velocity maximumVelocity)
        {
            this.PathReconstructor.SetCameFrom(current, via); // 修改前驱节点

            var node = new PathFinderNode(current, costSoFar, ExpectedDuration(current, goal, maximumVelocity));
            this.Interesting.Insert(node);
            this.Nodes[current] = node;
        }

        /// <summary>
        /// 节点a走到节点b需要的时间
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="maximumVelocity"></param>
        /// <returns></returns>
        public static Duration ExpectedDuration(INode a, INode b, Velocity maximumVelocity)
            => Distance.BeweenPositions(a.Position, b.Position) / maximumVelocity;
    }
}
