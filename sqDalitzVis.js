class Kinematics {

  constructor(mP, m1, m2, m3) {

    this.mP = mP;
    this.m1 = m1;
    this.m2 = m2;
    this.m3 = m3;

    this.mPSq = mP ** 2;

    this.m13Sq = 0;
    this.m13 = 0;

    this.m23Sq = 0;
    this.m23 = 0;

    this.m12Sq = 0;
    this.m12 = 0;

    this.mij = [this.m23, this.m13, this.m12];
    this.mijSq = [this.m23Sq, this.m13Sq, this.m12Sq];
    this.mSq = [this.m1 ** 2, this.m2 ** 2, this.m3 ** 2]

    this.mDTot = m1 + m2 + m3;
    this.mSqDTot = m1*m1 + m2*m2 + m3*m3;

    this.mMin = [this.mDTot - this.m1,
                 this.mDTot - this.m2,
                 this.mDTot - this.m3,];

    this.mMax = [this.mP - this.m1,
                this.mP - this.m2,
                this.mP - this.m3,];

    this.mDiff = [this.mMax[0] - this.mMin[0],
                  this.mMax[1] - this.mMin[1],
                  this.mMax[2] - this.mMin[2],];

    this.cij = [0, 0, 0];

  }

  updateKinematics(m13Sq, m23Sq) {
    this.updateMassSquares(m13Sq, m23Sq)
    this.calcHelicities()
    this.calcSqDPVars()
  }

  updateMassSquares(m13Sq, m23Sq) {

    this.m13 = Math.sqrt(m13Sq);
    this.m13Sq = m13Sq;

    this.m23 = Math.sqrt(m23Sq);
    this.m23Sq = m23Sq;

    this.m12Sq = this.calcThirdMassSq(this.m13Sq, this.m23Sq);
    this.m12 = Math.sqrt(this.m12Sq);

    this.mij = [this.m23, this.m13, this.m12];
    this.mijSq = [this.m23Sq, this.m13Sq, this.m12Sq];
    console.log(this.mij, this.mijSq, )

  }

  calcThirdMassSq(first, second) {
    return this.mPSq + this.mSqDTot - first - second;
  }

  e_i_CMS_ij(i, j, k) {
    return (this.mijSq[k] - this.mSq[j] + this.mSq[i]) / (2. * this.mij[k]);
  }

  e_k_CMS_ij(i, j, k) {
    return (this.mPSq - this.mijSq[k] - this.mSq[k]) / (2. * this.mij[k]);
  }

  pCalc(e, m2) {
    return Math.sqrt(e * e - m2);
  }

  calcHelicities(i, j, k) {

    this.cij[2] = this.cFromM(0, 1, 2);
    this.cij[0] = this.cFromM(1, 2, 0);
    this.cij[1] = this.cFromM(2, 0, 1);

  }

  mFromC(i, j, k) {
    ei = this.e_i_CMS_ij(i, j, k);
    ek = this.e_k_CMS_ij(i, j, k);

    qi = this.pCalc(ei, this.mSq[i]);
    qk = this.pCalc(ek, this.mSq[k]);

    return this.mSq[i] + this.mSq[k] + 2 * ei * ek - 2 * qi * qk * this.cij[k];
  }

  cFromM(i, j, k) {
    var ei = this.e_i_CMS_ij(i, j, k);
    var ek = this.e_k_CMS_ij(i, j, k);

    var qi = this.pCalc(ei, this.mSq[i]);
    var qk = this.pCalc(ek, this.mSq[k]);

    var c = -1 * (this.mijSq[j] - this.mSq[i] - this.mSq[k] - 2 * ei * ek) / (2 * qi * qk);

    if (i == 1) {
      c *= -1;
    }

    return c

  }

  calcSqDPVars() {
    var val = 2 * (this.mij[2] - this.mMin[2]) / this.mDiff[2] - 1;
    console.log(val, this.mij[2], this.cij[2], (1./Math.PI) * Math.acos(val))
    this.mPrime = (1./Math.PI) * Math.acos(val);
    this.thPrime = (1./Math.PI) * Math.acos(this.cij[2]);
  }

  scale(fromMin, fromMax, toMin, toMax, v) {
    return toMin + ((toMax - toMin) / (fromMax - fromMin)) * (v - fromMin)
  }

}

(function() {

  var isImg1Loaded;
  var isImg2Loaded;

  window.addEventListener('load', function() {
    document.getElementById('input1').addEventListener('change', function() {
        if (this.files && this.files[0]) {
            var img = document.getElementById('myImg1');
            img.src = URL.createObjectURL(this.files[0]);
            src = img.src;
            img.onload = image1Loaded;
        }
    });
  });

  window.addEventListener('load', function() {
    document.getElementById('input2').addEventListener('change', function() {
        if (this.files && this.files[0]) {
            var img = document.getElementById('myImg2');
            img.src = URL.createObjectURL(this.files[0]);
            src = img.src;
            img.onload = image2Loaded;
        }
    });
  });

  function imagesLoaded() {
    var container = document.getElementById('canvas');
    var container2 = document.getElementById('canvas2');
    init2(container, container2, 400, 400, '#ddd');
  }

  function image1Loaded() {
    isImg1Loaded = true;
    if (isImg1Loaded && isImg2Loaded) {
      imagesLoaded()
    }
  }
  function image2Loaded() {
    isImg2Loaded = true;
    if (isImg1Loaded && isImg2Loaded) {
      imagesLoaded()
    }
  }

    function createCanvas(parent, width, height, fillColor, bkg) {
        var canvas = {};
        canvas.node = document.createElement('canvas');
        canvas.context = canvas.node.getContext('2d');
        canvas.node.width = width || 100;
        canvas.node.height = height || 100;

        var ctx = canvas.context;

        // define a custom fillCircle method
        ctx.fillCircle = function(x, y, radius, fillColor) {
            this.fillStyle = fillColor;
            this.beginPath();
            this.moveTo(x, y);
            this.arc(x, y, radius, 0, Math.PI * 2, false);
            this.fill();
        };

        canvas.node.onmousedown = function(e) {
            canvas.isDrawing = true;
        };
        canvas.node.onmouseup = function(e) {
            canvas.isDrawing = false;
        };
        parent.appendChild(canvas.node);
        return canvas;
    }

    function init2(container, container2, width, height, fillColor) {

        var canvas1 = createCanvas(container, width, height, fillColor);
        var canvas2 = createCanvas(container2, width, height, fillColor);

        var ctx1 = canvas1.context;
        var ctx2 = canvas2.context;

        let bgImg1 = new Image();
        bgImg1 = document.getElementById('myImg1');
        ctx1.drawImage(bgImg1, 0, 0, width, height);

        let bgImg2 = new Image();
        bgImg2 = document.getElementById('myImg2');
        ctx2.drawImage(bgImg2, 0, 0, width, height);

        canvas1.node.onmousemove = function(e) {

            if (!canvas1.isDrawing) {
               return;
            }

            var x = e.pageX - this.offsetLeft;
            var y = e.pageY - this.offsetTop;

            var m13SqMin = kin.mMin[1] ** 2;
            var m13SqMax = kin.mMax[1] ** 2;

            var m23SqMin = kin.mMin[0] ** 2;
            var m23SqMax = kin.mMax[0] ** 2;

            var xMin = 0;
            var xMax = 400;

            var yMin = 0;
            var yMax = 400;

            var m13Sq = kin.scale(xMin, xMax, m13SqMin, m13SqMax, x)
            var m23Sq = kin.scale(xMin, xMax, m23SqMin, m23SqMax, y)

            // Origin is at top left
            m23Sq = m23SqMax - m23Sq

            kin.updateKinematics(m13Sq, m23Sq)

            var mPrime = kin.mPrime
            var thetaPrime = kin.thPrime

            var xPrime = kin.scale(0, 1, xMin, xMax, mPrime)
            var yPrime = kin.scale(0, 1, yMin, yMax, thetaPrime) // No folding

            var radius = 1; // or whatever
            var black = '#000000';
            var red = '#ff0000';

            console.log('x', x, ', y', y, ', m13Sq', m13Sq, ', m23Sq', m23Sq, ', mPrime', mPrime, ', thetaPrime', thetaPrime)

            ctx1.fillCircle(x, y, radius, black);
            // ctx2.fillCircle(3 * x, 3 * y, radius, red);
            ctx2.fillCircle(xPrime, yPrime, radius, red);
        };

        canvas2.node.onmousemove = function(e) {
            if (!canvas2.isDrawing) {
               return;
            }

            var x = e.pageX - this.offsetLeft;
            var y = e.pageY - this.offsetTop;

            var radius = 1; // or whatever
            var black = '#000000';
            var red = '#ff0000';
            ctx1.fillCircle(x / 3., y / 3., radius, black);
            ctx2.fillCircle(x, y, radius, red);
        };

    }

    var kin = new Kinematics(5.279, 0.494, 0.140, 0.140) // Kpipi

})();
