package me.drton.jmavsim.vehicle;
import java.util.Map;

public class APM {

    public double wing_span = 0.0;
    public double wing_area = 0.0;
    public double mean_aerodynamic_chord = 0.0;

    // Force and Moments coefficients at rest
    public double m_CD_0 = 0., m_CS_0 = 0., m_CL_0 = 0.; //!< \brief Resultant Aerodynamic Forces Coefficients clean configuration
    public double m_Cl_0 = 0., m_Cm_0 = 0., m_Cn_0 = 0.; //!< \brief Resultant Aerodynamic Moments Coefficients clean configuration

    // Force coefficients derivatives with respect to parameters of interest
    public double m_CD_alpha = 0.,m_CD_alpha2 = 0., m_CD_beta = 0., m_CD_beta2 = 0., m_CD_q = 0., m_CD_delta_e2 = 0.;
    public double m_CL_alpha = 0., m_CL_q = 0., m_CL_delta_e = 0.;
    public double m_CS_beta = 0., m_CS_p = 0., m_CS_r = 0., m_CS_delta_a = 0., m_CS_delta_r = 0.;

    // Moments coefficients derivatives with respect to parameters of interest
    public double m_Cl_beta = 0., m_Cl_p = 0., m_Cl_r = 0., m_Cl_delta_a = 0., m_Cl_delta_r = 0.;
    public double m_Cm_alpha = 0., m_Cm_delta_e = 0., m_Cm_q = 0.;
    public double m_Cn_beta = 0., m_Cn_p = 0., m_Cn_r = 0., m_Cn_delta_a = 0., m_Cn_delta_r = 0.;

    public APM(Map<String, Double> m) {
        this.wing_span = m.containsKey("wing_span") ? m.get("wing_span") : 0.0;
        this.wing_area = m.containsKey("wing_area") ? m.get("wing_area") : 0.0;
        this.mean_aerodynamic_chord = m.containsKey("mean_aerodynamic_chord") ? m.get("mean_aerodynamic_chord") : 0.0;
        this.m_CD_0 = m.containsKey("m_CD_0") ? m.get("m_CD_0") : 0.0;
        this.m_CS_0 = m.containsKey("m_CS_0") ? m.get("m_CS_0") : 0.0;
        this.m_CL_0 = m.containsKey("m_CL_0") ? m.get("m_CL_0") : 0.0;
        this.m_Cl_0 = m.containsKey("m_Cl_0") ? m.get("m_Cl_0") : 0.0;
        this.m_Cm_0 = m.containsKey("m_Cm_0") ? m.get("m_Cm_0") : 0.0;
        this.m_Cn_0 = m.containsKey("m_Cn_0") ? m.get("m_Cn_0") : 0.0;
        this.m_CD_alpha = m.containsKey("m_CD_alpha") ? m.get("m_CD_alpha") : 0.0;
        this.m_CD_alpha2 = m.containsKey("m_CD_alpha2") ? m.get("m_CD_alpha2") : 0.0;
        this.m_CD_beta = m.containsKey("m_CD_beta") ? m.get("m_CD_beta") : 0.0;
        this.m_CD_beta2 = m.containsKey("m_CD_beta2") ? m.get("m_CD_beta2") : 0.0;
        this.m_CD_q = m.containsKey("m_CD_q") ? m.get("m_CD_q") : 0.0;
        this.m_CD_delta_e2 = m.containsKey("m_CD_delta_e2") ? m.get("m_CD_delta_e2") : 0.0;
        this.m_CL_alpha = m.containsKey("m_CL_alpha") ? m.get("m_CL_alpha") : 0.0;
        this.m_CL_q = m.containsKey("m_CL_q") ? m.get("m_CL_q") : 0.0;
        this.m_CL_delta_e = m.containsKey("m_CL_delta_e") ? m.get("m_CL_delta_e") : 0.0;
        this.m_CS_beta = m.containsKey("m_CS_beta") ? m.get("m_CS_beta") : 0.0;
        this.m_CS_p = m.containsKey("m_CS_p") ? m.get("m_CS_p") : 0.0;
        this.m_CS_r = m.containsKey("m_CS_r") ? m.get("m_CS_r") : 0.0;
        this.m_CS_delta_a = m.containsKey("m_CS_delta_a") ? m.get("m_CS_delta_a") : 0.0;
        this.m_CS_delta_r = m.containsKey("m_CS_delta_r") ? m.get("m_CS_delta_r") : 0.0;
        this.m_Cl_beta = m.containsKey("m_Cl_beta") ? m.get("m_Cl_beta") : 0.0;
        this.m_Cl_p = m.containsKey("m_Cl_p") ? m.get("m_Cl_p") : 0.0;
        this.m_Cl_r = m.containsKey("m_Cl_r") ? m.get("m_Cl_r") : 0.0;
        this.m_Cl_delta_a = m.containsKey("m_Cl_delta_a") ? m.get("m_Cl_delta_a") : 0.0;
        this.m_Cl_delta_r = m.containsKey("m_Cl_delta_r") ? m.get("m_Cl_delta_r") : 0.0;
        this.m_Cm_alpha = m.containsKey("m_Cm_alpha") ? m.get("m_Cm_alpha") : 0.0;
        this.m_Cm_delta_e = m.containsKey("m_Cm_delta_e") ? m.get("m_Cm_delta_e") : 0.0;
        this.m_Cm_q = m.containsKey("m_Cm_q") ? m.get("m_Cm_q") : 0.0;
        this.m_Cn_beta = m.containsKey("m_Cn_beta") ? m.get("m_Cn_beta") : 0.0;
        this.m_Cn_p = m.containsKey("m_Cn_p") ? m.get("m_Cn_p") : 0.0;
        this.m_Cn_r = m.containsKey("m_Cn_r") ? m.get("m_Cn_r") : 0.0;
        this.m_Cn_delta_a = m.containsKey("m_Cn_delta_a") ? m.get("m_Cn_delta_a") : 0.0;
        this.m_Cn_delta_r = m.containsKey("m_Cn_delta_r") ? m.get("m_Cn_delta_r") : 0.0;
    }

    public String describe() {
        StringBuilder builder = new StringBuilder();
        builder.append("wing_span: " + this.wing_span);
        builder.append("wing_area: " + this.wing_area);
        builder.append("mean_aerodynamic_chord: " + this.mean_aerodynamic_chord);
        builder.append("m_CD_0: " + this.m_CD_0);
        builder.append("m_CS_0: " + this.m_CS_0);
        builder.append("m_CL_0: " + this.m_CL_0);
        builder.append("m_Cl_0: " + this.m_Cl_0);
        builder.append("m_Cm_0: " + this.m_Cm_0);
        builder.append("m_Cn_0: " + this.m_Cn_0);
        builder.append("m_CD_alpha: " + this.m_CD_alpha);
        builder.append("m_CD_alpha2: " + this.m_CD_alpha2);
        builder.append("m_CD_beta: " + this.m_CD_beta);
        builder.append("m_CD_beta2: " + this.m_CD_beta2);
        builder.append("m_CD_q: " + this.m_CD_q);
        builder.append("m_CD_delta_e2: " + this.m_CD_delta_e2);
        builder.append("m_CL_alpha: " + this.m_CL_alpha);
        builder.append("m_CL_q: " + this.m_CL_q);
        builder.append("m_CL_delta_e: " + this.m_CL_delta_e);
        builder.append("m_CS_beta: " + this.m_CS_beta);
        builder.append("m_CS_p: " + this.m_CS_p);
        builder.append("m_CS_r: " + this.m_CS_r);
        builder.append("m_CS_delta_a: " + this.m_CS_delta_a);
        builder.append("m_CS_delta_r: " + this.m_CS_delta_r);
        builder.append("m_Cl_beta: " + this.m_Cl_beta);
        builder.append("m_Cl_p: " + this.m_Cl_p);
        builder.append("m_Cl_r: " + this.m_Cl_r);
        builder.append("m_Cl_delta_a: " + this.m_Cl_delta_a);
        builder.append("m_Cl_delta_r: " + this.m_Cl_delta_r);
        builder.append("m_Cm_alpha: " + this.m_Cm_alpha);
        builder.append("m_Cm_delta_e: " + this.m_Cm_delta_e);
        builder.append("m_Cm_q: " + this.m_Cm_q);
        builder.append("m_Cn_beta: " + this.m_Cn_beta);
        builder.append("m_Cn_p: " + this.m_Cn_p);
        builder.append("m_Cn_r: " + this.m_Cn_r);
        builder.append("m_Cn_delta_a: " + this.m_Cn_delta_a);
        builder.append("m_Cn_delta_r: " + this.m_Cn_delta_r);
        return builder.toString();
    }
}