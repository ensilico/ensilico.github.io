function testLerp(subject, selftest) {
  var small = function () {
    return Math.pow(2, -53); // or whatever
  };

  var lower = function (subj, pred) {
    pred.equal(small(), subj(small(), 1, 0));
  }

  var upper = function (subj, pred) {
    pred.equal(small(), subj(1, small(), 1));
  }

  var naive1 = function (f1, f2, t) {
    return f1 + t * (f2 - f1);
  };

  var naive2 = function (f1, f2, t) {
    return (1 - t) * (f1 - f2) + f2;
  };

  lower(naive1, selftest.refute);
  lower(subject, selftest.assert);
  upper(naive2, selftest.refute);
  upper(subject, selftest.assert);
  selftest.assert.equal(0.5, subject(0, 1, 0.5));
}

Scalar.lerp = Selftest
  .run(testLerp)
  .against(function (f1, f2, t) {
    return (1 - t) * f1 + t * f2;
  });
