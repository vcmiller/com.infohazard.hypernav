using System;
using System.Collections.Generic;

namespace HyperNav.Runtime.Utility {
    public class Heap<T> {
        private readonly List<(T item, float priority)> _items = new List<(T item, float priority)>();

        public int Count => _items.Count;
        
        public void Clear() => _items.Clear();

        public void Add(T item, float priority) {
            _items.Add((item, priority));
            BubbleUp(_items.Count - 1);
        }

        public void Update(T item, float newPriority) {
            int i = _items.FindIndex(t => Equals(t.item, item));
            if (i < 0) throw new ArgumentException("Value not in heap.");
            var val = _items[i];
            float oldPriority = val.priority;
            val.priority = newPriority;
            _items[i] = val;
            if (newPriority > oldPriority) {
                BubbleUp(i);
            } else if (newPriority < oldPriority) {
                BubbleDown(i);
            }
        }

        public T Peek() {
            if (_items.Count > 0) {
                return _items[0].item;
            } else {
                throw new Exception("Heap is empty.");
            }
        }

        public T Remove() {
            T max = Peek();
            _items[0] = _items[_items.Count - 1];
            _items.RemoveAt(_items.Count - 1);

            BubbleDown(0);
            return max;
        }

        private void Swap(int index1, int index2) {
            (_items[index1], _items[index2]) = (_items[index2], _items[index1]);
        }

        private void BubbleDown(int index) {
            while (index < _items.Count) {
                int leftIndex = index * 2 + 1;
                int rightIndex = index * 2 + 2;

                int largest = index;
                if (leftIndex < _items.Count && _items[leftIndex].priority > _items[largest].priority)
                    largest = leftIndex;
                if (rightIndex < _items.Count && _items[rightIndex].priority > _items[largest].priority)
                    largest = rightIndex;

                if (largest != index) {
                    Swap(largest, index);
                    index = largest;
                } else {
                    break;
                }
            }
        }

        private void BubbleUp(int index) {
            while (index > 0) {
                int parentIndex = (index - 1) / 2;
                if (_items[index].priority > _items[parentIndex].priority) {
                    Swap(index, parentIndex);
                    index = parentIndex;
                } else {
                    break;
                }
            }
        }
    }
}
