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

  updateMassSquaresSDP(mPrime, thPrime) {

    this.mPrime = mPrime;
    this.thPrime = thPrime;

    var m12 = 0.5 * this.mDiff[2] * ( 1.0 + Math.cos( Math.PI * mPrime ) ) + this.mMin[2];
    var c12 = Math.cos( Math.PI * thPrime );

    this.updateMassSquaresSDP_12(m12, c12)

  }

  updateMassSquaresSDP_12(m12, c12) {

    this.mij[2] = m12
    this.mijSq[2] = m12*m12
    this.cij[2] = c12

    this.mijSq[1] = this.mFromC( 0, 1, 2 )
    this.mij[1] = Math.sqrt( this.mijSq[1] )

    this.mijSq[0] = this.calcThirdMassSq( this.mijSq[2], this.mijSq[1] )
    this.mij[0] = Math.sqrt( this.mijSq[0] )

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

    var ei = this.e_i_CMS_ij(i, j, k);
    var ek = this.e_k_CMS_ij(i, j, k);

    var qi = this.pCalc(ei, this.mSq[i]);
    var qk = this.pCalc(ek, this.mSq[k]);

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

    this.mPrime = (1./Math.PI) * Math.acos(val);
    this.thPrime = (1./Math.PI) * Math.acos(this.cij[2]);
  }

  scale(fromMin, fromMax, toMin, toMax, v) {
    return toMin + ((toMax - toMin) / (fromMax - fromMin)) * (v - fromMin)
  }

}

var mP = 5.279;
var m1 = 0.494;
var m2 = 0.140;
var m3 = 0.140;
var kin = new Kinematics(mP, m1, m2, m3);

var fold = false;

(function() {

  var isImg1Loaded;
  var isImg2Loaded;

  var xPrev = -1;
  var yPrev = -1;
  var xPrimePrev = -1;
  var yPrimePrev = -1;

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

        canvas.node.onmousedown = function(e) {
            canvas.isDrawing = true;
        };
        canvas.node.onmouseup = function(e) {
            canvas.isDrawing = false;
            xPrev = -1;
            xPrimePrev = -1;
        };
        parent.appendChild(canvas.node);
        return canvas;
    }

    function getInput() {

      var mPField = document.getElementById("mP").value;
      var m1Field = document.getElementById("m1").value;
      var m2Field = document.getElementById("m2").value;
      var m3Field = document.getElementById("m3").value;

      fold = document.getElementById("fold").checked;

      if (mPField != "") {
        mP = parseFloat(mPField);
      }

      if (m1Field != "") {
        m1 = parseFloat(m1Field);
      }

      if (m2Field != "") {
        m2 = parseFloat(m2Field);
      }

      if (m3Field != "") {
        m3 = parseFloat(m3Field);
      }

      kin = new Kinematics(mP, m1, m2, m3);
    }

    function init2(container, container2, width, height, fillColor) {

        var canvas1 = createCanvas(container, width, height, fillColor);
        var canvas2 = createCanvas(container2, width, height, fillColor);

        getInput();

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

            // Origin is at top left
            var m13Sq = kin.scale(xMin, xMax, m13SqMin, m13SqMax, x);
            var m23Sq = kin.scale(yMin, yMax, m23SqMax, m23SqMin, y);

            kin.updateKinematics(m13Sq, m23Sq);

            var mPrime = kin.mPrime;
            var thetaPrime = kin.thPrime;

            var xPrime = kin.scale(0, 1, xMin, xMax, mPrime);

            var yPrime = kin.scale(0, 1, yMax, yMin, thetaPrime);
            if (fold) {
              yPrime = kin.scale(0, 0.5, yMax, yMin, thetaPrime);
            }

            var black = '#000000';
            var red = '#ff0000';

            if (xPrev != -1) {
              ctx1.beginPath();
              ctx1.moveTo(xPrev, yPrev);
              ctx1.lineTo(x, y);
              ctx1.strokeStyle = black;
              ctx1.stroke();

              ctx2.beginPath();
              ctx2.moveTo(xPrimePrev, yPrimePrev);
              ctx2.lineTo(xPrime, yPrime);
              ctx2.strokeStyle = red;
              ctx2.stroke();
            }

            xPrev = x;
            yPrev = y;

            xPrimePrev = xPrime;
            yPrimePrev = yPrime;
        };

        canvas2.node.onmousemove = function(e) {

            if (!canvas2.isDrawing) {
               return;
            }

            var xPrime = e.pageX - this.offsetLeft;
            var yPrime = e.pageY - this.offsetTop;

            var xPrimeMin = 0;
            var xPrimeMax = 400;

            var yPrimeMin = 0;
            var yPrimeMax = 400;

            var mPrime = kin.scale(xPrimeMin, xPrimeMax, 0, 1, xPrime);
            var thPrime = kin.scale(yPrimeMin, yPrimeMax, 1, 0, yPrime);
            if (fold) {
              thPrime = kin.scale(yPrimeMin, yPrimeMax, 0.5, 0, yPrime);
            }

            kin.updateMassSquaresSDP(mPrime, thPrime);

            var m13Sq = kin.mijSq[1];
            var m23Sq = kin.mijSq[0];

            var m13SqMin = kin.mMin[1] ** 2;
            var m13SqMax = kin.mMax[1] ** 2;

            var m23SqMin = kin.mMin[0] ** 2;
            var m23SqMax = kin.mMax[0] ** 2;


            // Origin is at top left
            var x = kin.scale(m13SqMin, m13SqMax, xPrimeMin, xPrimeMax, m13Sq)
            var y = kin.scale(m23SqMin, m23SqMax, yPrimeMax, yPrimeMin, m23Sq) // No folding

            var black = '#000000';
            var red = '#ff0000';

            if (xPrimePrev != -1) {

              ctx2.beginPath();
              ctx2.moveTo(xPrimePrev, yPrimePrev);
              ctx2.lineTo(xPrime, yPrime);
              ctx2.strokeStyle = black;
              ctx2.stroke();

              ctx1.beginPath();
              ctx1.moveTo(xPrev, yPrev);
              ctx1.lineTo(x, y);
              ctx1.strokeStyle = red;
              ctx1.stroke();
            }

            xPrev = x;
            yPrev = y;

            xPrimePrev = xPrime;
            yPrimePrev = yPrime;

        };
    }

})();
