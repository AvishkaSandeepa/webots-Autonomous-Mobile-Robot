## ❄ Anchor points of the rear wheels

### Mathematical approach to get the exact place
1. Don't fix the endpoint solid coordinates first!
2. Choose and set a proper anchor point. Do not change it hereafter.
3. Change the translation coordinates of the wheel to translational coordinates of the anchor point.
4. Now the origin of the wheel will be placed at the anchor point!
5. Our target is to get the axis of the wheel to the anchor point.
6. We know the dimensions of the wheel and the scaling factor we used when importing solidWorks model into the webots.
7. Now it's all about finding the correct combination of these values!

***My Calculations:*** 😁


<img width="400px" src="figures/anchor.png" align="center"/>

* Anchor point (0.74, 0.36, 0) in meters
* Dimensions of the wheel: Diameter 6.5 cm -> 0.0325 m radius!
* Set the translation coordinates of the wheel to coordinates  of point A.
* A = (0.74 +- 0.0325x, 0.36 +- 0.0325x, 0)
* For me A was equals to 0.74 - 0.0325 x 5, 0.36 + 0.0325 x 5, 0) where x = 5 which was obtained by trial and error.
* Side note: scaling factor I used when importing solidWorks model into the webots = 5/1000;
* This may be the reason for x to be equal to 5! Anyway not sure about it.

