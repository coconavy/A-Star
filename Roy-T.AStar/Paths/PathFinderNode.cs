using System;
using Roy_T.AStar.Graphs;
using Roy_T.AStar.Primitives;

namespace Roy_T.AStar.Paths
{
    internal sealed class PathFinderNode : IComparable<PathFinderNode>
    {
        public PathFinderNode(INode node, Duration durationSoFar, Duration expectedRemainingTime)
        {
            this.Node = node;
            this.DurationSoFar = durationSoFar;
            this.ExpectedRemainingTime = expectedRemainingTime;
            this.ExpectedTotalTime = this.DurationSoFar + this.ExpectedRemainingTime;
        }

        public INode Node { get; } // 节点
        public Duration DurationSoFar { get; } //已经持续时间
        public Duration ExpectedRemainingTime { get; } // 剩余时间
        public Duration ExpectedTotalTime { get; } // 期望总时间

        public int CompareTo(PathFinderNode other) => this.ExpectedTotalTime.CompareTo(other.ExpectedTotalTime);

        // Node.Position.X, Node.Position.Y指到达的目标点，已经经过这个目标点走到终点所需的总时间
        public override string ToString() => $"📍{{{this.Node.Position.X}, {this.Node.Position.Y}}}, ⏱~{this.ExpectedTotalTime}";
    }
}
