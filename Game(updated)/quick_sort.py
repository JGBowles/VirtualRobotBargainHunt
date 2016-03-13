def quickSort(arr):
    lower = []
    pivotList = []
    greater = []
    if len(arr) <= 1:
        return arr
    else:
        pivot = arr[0]
        for i in arr:
            if i < pivot:
                lower.append(i)
            elif i > pivot:
                greater.append(i)
            else:
                pivotList.append(i)
        lower = quickSort(lower)
        greater = quickSort(greater)
        return lower + pivotList + greater
 
aList = ['fuel','battery','door','cpu','antenna','propulsion']    
aList = quickSort(aList)
print(aList)
