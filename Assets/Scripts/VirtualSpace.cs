using UnityEngine;

public class VirtualSpace : MonoBehaviour{
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(default, new Vector3(5f, 0, 5f));
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(default, new Vector3(1f, 0, 1f));
    }
}