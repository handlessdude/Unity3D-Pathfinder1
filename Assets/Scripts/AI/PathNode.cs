using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathNode //: MonoBehaviour
{
    public bool walkable;           //  Свободна для перемещения
    public Vector3 worldPosition;   //  Позиция в глобальных координатах
    private GameObject objPrefab;   //  Шаблон объекта
    public GameObject body;         //  Объект для отрисовки
    
    private PathNode parentNode = null;               //  откуда пришли
    
    /// <summary>
    /// Родительская вершина - предшествующая текущей в пути от начальной к целевой
    /// </summary>
    public PathNode ParentNode
    {
        get => parentNode;
        set => SetParent(value);
    }

    private float _distanceToStart = float.PositiveInfinity;  //  расстояние от начальной вершины

    /// <summary>
    /// Расстояние от начальной вершины до текущей (+infinity если ещё не развёртывали)
    /// </summary>
    public float DistanceToStart
    {
        get => _distanceToStart;
        set => _distanceToStart = value;
    }

    /// <summary>
    /// Устанавливаем родителя и обновляем расстояние от него до текущей вершины. Неоптимально - дважды расстояние считается
    /// </summary>
    /// <param name="parent"></param>
    private void SetParent(PathNode parent)
    {
        //  Указываем родителя
        parentNode = parent;
        //  Вычисляем расстояние
        if (parent == null)
        {
            _distanceToStart = float.PositiveInfinity;
            return;
        }
        _distanceToStart = parent.DistanceToStart + Vector3.Distance(body.transform.position, parent.body.transform.position);
    }

    /// <summary>
    /// Конструктор вершины
    /// </summary>
    /// <param name="_objPrefab">объект, который визуализируется в вершине</param>
    /// <param name="_walkable">проходима ли вершина</param>
    /// <param name="position">мировые координаты</param>
    public PathNode(GameObject _objPrefab, bool _walkable, Vector3 position)
    {
        objPrefab = _objPrefab;
        walkable = _walkable;
        worldPosition = position;
        body = GameObject.Instantiate(objPrefab, worldPosition, Quaternion.identity);
    }

    private const float HeightDeltaWeight = 40;
    /// <summary>
    /// Расстояние между вершинами с учетом разброса по высоте
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <param name="heightMultiplier">height delta mutiplier</param>
    /// <returns></returns>
    public static float HeightPenaltyDist(PathNode a, PathNode b)
    {
        return EuclideanDist(a,  b) + HeightPenalty(a, b);
    }
    
    /// <summary>
    /// Разница высот двух вершин
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <param name="heightMultiplier">height delta mutiplier</param>
    /// <returns></returns>
    public static float HeightPenalty(PathNode a, PathNode b, float heightMultiplier = HeightDeltaWeight)
    {
        return heightMultiplier * Mathf.Abs(a.body.transform.position.y - b.body.transform.position.y);
    }
    
    /// <summary>
    /// Евклидово Расстояние между вершинами
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    public static float EuclideanDist(PathNode a, PathNode b)
    {
        return Vector3.Distance(a.body.transform.position, b.body.transform.position);
    }
    
    /// <summary>
    /// Подсветить вершину - перекрасить в зеленый
    /// </summary>
    public void Illuminate()
    {
        body.GetComponent<Renderer>().material.color = Color.blue;
    }
    
    /// <summary>
    /// Подсветить вершину - перекрасить в зеленый
    /// </summary>
    public void Highlight()
    {
        body.GetComponent<Renderer>().material.color = Color.green;
    }
    
    /// <summary>
    /// Снять подсветку с вершины - перекрасить в синий
    /// </summary>
    public void Fade()
    {
        body.GetComponent<Renderer>().material.color = Color.gray;
    }
}
