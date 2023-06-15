using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class ArrayExtensions {

	public static void ResetArray<T> (this T[] array) {
		for (int i = 0; i < array.Length; i++) {
			array[i] = default(T);
		}
	}
}
