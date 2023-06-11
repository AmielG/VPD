using System.Collections;
using System.Collections.Generic;
using UnityEngine;


 public class CustomDictionary<TKey, TValue> : Dictionary<TKey, TValue>
  {
    public int MaxItemsToHold { get; set; }

    private Queue<TKey> orderedKeys = new Queue<TKey>();

    public new void Add(TKey key, TValue value)
    {
      orderedKeys.Enqueue(key);
      if (this.MaxItemsToHold != 0 && this.Count >= MaxItemsToHold)
      {
        this.Remove(orderedKeys.Dequeue());
      }

      base.Add(key, value);
    }
  }