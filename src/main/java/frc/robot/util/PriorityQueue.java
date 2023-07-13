package frc.robot.util;

import java.util.ArrayList;

class QueueNode<T> {
    T data;
    double cost;

    QueueNode(T data, double cost) {
        this.data = data;
        this.cost = cost;
    }
}

public class PriorityQueue<T> {
    private ArrayList<QueueNode<T>> nodes;

    public PriorityQueue() {
        nodes = new ArrayList<>();
    }

    public void add(T data, double cost) {
        nodes.add(new QueueNode<>(data, cost));
    }

    public T remove() {
        double lowest = nodes.get(0).cost;
        int lowestIndex = 0;

        for (int i = 0; i < nodes.size(); i++) {
            if (nodes.get(i).cost < lowest) {
                lowest = nodes.get(i).cost;
                lowestIndex = i;
            }
        }


        return nodes.remove(lowestIndex).data;
    }

    public boolean isEmpty() {
        return nodes.isEmpty();
    }
}