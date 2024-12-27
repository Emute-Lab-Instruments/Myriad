

panelThickness = 2;
panelHp=16;
holeCount=4;
holeWidth = 5.08 * 1.4; 


threeUHeight = 133.35; //overall 3u height
panelOuterHeight =128.5;
railClearance = 11.675;
panelInnerHeight = threeUHeight - (2 * railClearance); //rail clearance = ~11.675mm, top and bottom
railHeight = (threeUHeight-panelOuterHeight)/2;
mountSurfaceHeight = (panelOuterHeight-panelInnerHeight-railHeight*2)/2;

echo("mountSurfaceHeight",mountSurfaceHeight);

hp=5.08;
mountHoleDiameter = 3.2;
mountHoleRad =mountHoleDiameter/2;
hwCubeWidth = holeWidth-mountHoleDiameter;

offsetToMountHoleCenterY=mountSurfaceHeight/2;
offsetToMountHoleCenterX = hp - hwCubeWidth/2; // 1 hp from side to center of hole

echo(offsetToMountHoleCenterY);
echo(offsetToMountHoleCenterX);

module eurorackPanel(panelHp,  mountHoles=2, hw = holeWidth, ignoreMountHoles=false)
{
    //mountHoles ought to be even. Odd values are -=1
    difference()
    {
        cube([hp*panelHp,panelOuterHeight,panelThickness]);
        
        if(!ignoreMountHoles)
        {
            eurorackMountHoles(panelHp, mountHoles, holeWidth);
        }
    }
}

module eurorackMountHoles(php, holes, hw)
{
    holes = holes-holes%2;
    eurorackMountHolesTopRow(php, hw, holes/2);
    eurorackMountHolesBottomRow(php, hw, holes/2);
}

module eurorackMountHolesTopRow(php, hw, holes)
{
    
    //topleft
    translate([offsetToMountHoleCenterX,panelOuterHeight-offsetToMountHoleCenterY,0])
    {
        eurorackMountHole(hw);
    }
    if(holes>1)
    {
        translate([(hp*php)-hwCubeWidth-hp,panelOuterHeight-offsetToMountHoleCenterY,0])
    {
        eurorackMountHole(hw);
    }
    }
    if(holes>2)
    {
        holeDivs = php*hp/(holes-1);
        for (i =[1:holes-2])
        {
            translate([holeDivs*i,panelOuterHeight-offsetToMountHoleCenterY,0]){
                eurorackMountHole(hw);
            }
        }
    }
}

module eurorackMountHolesBottomRow(php, hw, holes)
{
    
    //bottomRight
    translate([(hp*php)-hwCubeWidth-hp,offsetToMountHoleCenterY,0])
    {
        eurorackMountHole(hw);
    }
    if(holes>1)
    {
        translate([offsetToMountHoleCenterX,offsetToMountHoleCenterY,0])
    {
        eurorackMountHole(hw);
    }
    }
    if(holes>2)
    {
        holeDivs = php*hp/(holes-1);
        for (i =[1:holes-2])
        {
            translate([holeDivs*i,offsetToMountHoleCenterY,0]){
                eurorackMountHole(hw);
            }
        }
    }
}

module eurorackMountHole(hw)
{
    
    mountHoleDepth = panelThickness+2; //because diffs need to be larger than the object they are being diffed from for ideal BSP operations
    
    if(hwCubeWidth<0)
    {
        hwCubeWidth=0;
    }
    translate([0,0,-1]){
    union()
    {
        cylinder(r=mountHoleRad, h=mountHoleDepth, $fn=20);
        translate([0,-mountHoleRad,0]){
        cube([hwCubeWidth, mountHoleDiameter, mountHoleDepth]);
        }
        translate([hwCubeWidth,0,0]){
            cylinder(r=mountHoleRad, h=mountHoleDepth, $fn=20);
            }
    }
}
}











///////////////////////////////

w = hp*panelHp;
ledholeRadius =2.55;

leftoffset = 154.83;
topoffset=37.75;

module hollowCylinder(ch, cr1, cr2, th) {
    difference(){
    cylinder(10, cr1, cr2, center=true, $fn=16);
    cylinder(10, cr1-th, cr2-th, center=true, $fn=16);
    }
}



module drawPanelWithHoles() {
    rSocket=3.5;
    rSwitch=4.2;
    rMomentarySwitch=4;
    rLed = 2;
    rReset = 1.5;
    rRotaryEnc=4.1;
    
    difference() {
        eurorackPanel(panelHp, holeCount,holeWidth);

        base = ((panelOuterHeight -panelInnerHeight) / 2) +1;
        


        //rotary encs
        translate([166.5-leftoffset, base + 47.5 - topoffset, 0])
        cylinder(10, rRotaryEnc,rRotaryEnc, center=true, $fn=16);

        translate([196.8-leftoffset, base + 46.5 - topoffset, 0])
        cylinder(10, rRotaryEnc,rRotaryEnc, center=true, $fn=16);

        translate([224.49-leftoffset, base + 47.5 - topoffset, 0])
        cylinder(10, rRotaryEnc,rRotaryEnc, center=true, $fn=16);
        
        //tft 
        translate([195.7-leftoffset, base + 76.91-topoffset,0])
        cylinder(10, r=17, center=true);
        
        
        //pots

        translate([165.3 - leftoffset, base + 74.5 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);

        translate([167.51 - leftoffset, base + 95 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);

        translate([182.71 - leftoffset, base + 109.34 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);

        translate([207.31 - leftoffset, base + 109.34 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);
        


        translate([222.3 - leftoffset, base + 95 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);

        translate([224.5 - leftoffset, base + 74.39 - topoffset, 0])
        cylinder(10, rSwitch,rSwitch, center=true, $fn=16);


        
        //sockets
        translate([164.6-leftoffset, base + 130.1 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);

        translate([176.97-leftoffset, base + 134.13 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);

        translate([189.41-leftoffset, base + 137.66 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);

        translate([201.77-leftoffset, base + 137.7 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);

        translate([214.16-leftoffset, base + 134.13 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);

        translate([226.54-leftoffset, base + 130.1 - topoffset, 0])
        cylinder(10, rSocket,rSocket, center=true, $fn=16);



//        translate([w * 0.77, 10, 0.5]) rotate([0,0,180])
//            linear_extrude(4)
//                text("MEML", size = 8,font="Lato");
    }
    
}

projection(cut=true) 
drawPanelWithHoles();


